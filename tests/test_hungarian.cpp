#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hungarian.h"

namespace {

using tracking::assignment::AssignmentResult;
using tracking::assignment::CostMatrix;
using tracking::assignment::ValidityMask;
using tracking::assignment::solve;

struct Optimum {
    Optimum() = default;
    Optimum(std::size_t matches, double cost) : matchCount(matches), totalCost(cost) {}

    std::size_t matchCount = 0;
    double totalCost = std::numeric_limits<double>::infinity();
};

void enumerateOptimum(
    const CostMatrix& costs,
    const ValidityMask& validEdges,
    std::size_t track,
    std::vector<bool>& usedDetections,
    std::size_t matchCount,
    double totalCost,
    Optimum& optimum) {
    if (track == costs.rows) {
        if (matchCount > optimum.matchCount ||
            (matchCount == optimum.matchCount && totalCost < optimum.totalCost)) {
            optimum = Optimum{matchCount, totalCost};
        }
        return;
    }

    enumerateOptimum(
        costs,
        validEdges,
        track + 1,
        usedDetections,
        matchCount,
        totalCost,
        optimum);

    for (std::size_t detection = 0; detection < costs.columns; ++detection) {
        const std::size_t index = track * costs.columns + detection;
        if (!validEdges.values[index] || usedDetections[detection]) {
            continue;
        }

        usedDetections[detection] = true;
        enumerateOptimum(
            costs,
            validEdges,
            track + 1,
            usedDetections,
            matchCount + 1,
            totalCost + costs.values[index],
            optimum);
        usedDetections[detection] = false;
    }
}

Optimum bruteForceOptimum(const CostMatrix& costs, const ValidityMask& validEdges) {
    Optimum optimum;
    std::vector<bool> usedDetections(costs.columns, false);
    enumerateOptimum(costs, validEdges, 0, usedDetections, 0, 0.0, optimum);
    return optimum;
}

void expectCompleteCoverage(
    const AssignmentResult& result,
    const CostMatrix& costs,
    const ValidityMask& validEdges) {
    std::vector<bool> seenTracks(costs.rows, false);
    std::vector<bool> seenDetections(costs.columns, false);

    for (const auto& match : result.matches) {
        ASSERT_LT(match.first, costs.rows);
        ASSERT_LT(match.second, costs.columns);
        EXPECT_FALSE(seenTracks[match.first]);
        EXPECT_FALSE(seenDetections[match.second]);
        EXPECT_TRUE(validEdges.values[match.first * costs.columns + match.second]);
        seenTracks[match.first] = true;
        seenDetections[match.second] = true;
    }
    for (const std::size_t track : result.unmatchedTracks) {
        ASSERT_LT(track, costs.rows);
        EXPECT_FALSE(seenTracks[track]);
        seenTracks[track] = true;
    }
    for (const std::size_t detection : result.unmatchedDetections) {
        ASSERT_LT(detection, costs.columns);
        EXPECT_FALSE(seenDetections[detection]);
        seenDetections[detection] = true;
    }
    for (const bool seen : seenTracks) {
        EXPECT_TRUE(seen);
    }
    for (const bool seen : seenDetections) {
        EXPECT_TRUE(seen);
    }
}

void expectOptimal(
    const CostMatrix& costs,
    const ValidityMask& validEdges,
    const AssignmentResult& result) {
    expectCompleteCoverage(result, costs, validEdges);

    double resultCost = 0.0;
    for (const auto& match : result.matches) {
        resultCost += costs.values[match.first * costs.columns + match.second];
    }
    const Optimum optimum = bruteForceOptimum(costs, validEdges);
    EXPECT_EQ(result.matches.size(), optimum.matchCount);
    EXPECT_NEAR(resultCost, optimum.totalCost, 1.0e-6);
}

TEST(HungarianTest, SupportsEmptyDimensions) {
    const AssignmentResult empty = solve(CostMatrix{0, 0, {}}, ValidityMask{0, 0, {}});
    EXPECT_TRUE(empty.matches.empty());
    EXPECT_TRUE(empty.unmatchedTracks.empty());
    EXPECT_TRUE(empty.unmatchedDetections.empty());

    const CostMatrix noTracks{0, 2, {}};
    const ValidityMask noTracksMask{0, 2, {}};
    const AssignmentResult noTracksResult = solve(noTracks, noTracksMask);
    expectOptimal(noTracks, noTracksMask, noTracksResult);
    EXPECT_EQ(noTracksResult.unmatchedDetections, (std::vector<std::size_t>{0, 1}));

    const CostMatrix noDetections{3, 0, {}};
    const ValidityMask noDetectionsMask{3, 0, {}};
    const AssignmentResult noDetectionsResult = solve(noDetections, noDetectionsMask);
    expectOptimal(noDetections, noDetectionsMask, noDetectionsResult);
    EXPECT_EQ(noDetectionsResult.unmatchedTracks, (std::vector<std::size_t>{0, 1, 2}));
}

TEST(HungarianTest, FindsMaximumCardinalityThenMinimumCostForSquareAndRectangularMatrices) {
    const CostMatrix square{2, 2, {4.0F, 1.0F, 2.0F, 3.0F}};
    const ValidityMask squareMask{2, 2, {true, true, true, true}};
    expectOptimal(square, squareMask, solve(square, squareMask));

    const CostMatrix moreDetections{2, 3, {5.0F, 1.0F, 8.0F, 2.0F, 7.0F, 3.0F}};
    const ValidityMask moreDetectionsMask{2, 3, {true, true, true, true, true, true}};
    expectOptimal(moreDetections, moreDetectionsMask, solve(moreDetections, moreDetectionsMask));

    const CostMatrix moreTracks{3, 2, {1.0F, 5.0F, 2.0F, 1.0F, 4.0F, 2.0F}};
    const ValidityMask moreTracksMask{3, 2, {true, true, true, true, true, true}};
    expectOptimal(moreTracks, moreTracksMask, solve(moreTracks, moreTracksMask));
}

TEST(HungarianTest, RespectsInvalidEdgesAndAvoidsPostHocDeletionRegression) {
    const CostMatrix allForbidden{2, 3, {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F}};
    const ValidityMask noEdges{2, 3, {false, false, false, false, false, false}};
    const AssignmentResult noEdgesResult = solve(allForbidden, noEdges);
    expectOptimal(allForbidden, noEdges, noEdgesResult);

    const CostMatrix partial{2, 2, {0.0F, 10.0F, 10.0F, 0.0F}};
    const ValidityMask partialMask{2, 2, {false, true, true, true}};
    const AssignmentResult partialResult = solve(partial, partialMask);
    expectOptimal(partial, partialMask, partialResult);
    EXPECT_EQ(partialResult.matches,
              (std::vector<std::pair<std::size_t, std::size_t>>{{0, 1}, {1, 0}}));
}

TEST(HungarianTest, EqualCostsProduceDeterministicOrdering) {
    const CostMatrix costs{2, 2, {1.0F, 1.0F, 1.0F, 1.0F}};
    const ValidityMask validEdges{2, 2, {true, true, true, true}};

    const AssignmentResult first = solve(costs, validEdges);
    const AssignmentResult second = solve(costs, validEdges);
    expectOptimal(costs, validEdges, first);
    EXPECT_EQ(first.matches, second.matches);
    EXPECT_EQ(first.matches,
              (std::vector<std::pair<std::size_t, std::size_t>>{{0, 0}, {1, 1}}));
}

TEST(HungarianTest, RejectsMalformedMatricesAndNonFiniteValidCosts) {
    EXPECT_THROW(
        solve(CostMatrix{1, 1, {}}, ValidityMask{1, 1, {true}}),
        std::invalid_argument);
    EXPECT_THROW(
        solve(CostMatrix{1, 1, {1.0F}}, ValidityMask{1, 2, {true, true}}),
        std::invalid_argument);
    EXPECT_THROW(
        solve(CostMatrix{1, 2, {1.0F, 2.0F}}, ValidityMask{1, 2, {true}}),
        std::invalid_argument);
    EXPECT_THROW(
        solve(
            CostMatrix{1, 1, {std::numeric_limits<float>::quiet_NaN()}},
            ValidityMask{1, 1, {true}}),
        std::invalid_argument);
    EXPECT_THROW(
        solve(
            CostMatrix{1, 1, {std::numeric_limits<float>::infinity()}},
            ValidityMask{1, 1, {true}}),
        std::invalid_argument);

    const CostMatrix forbiddenNonFinite{1, 1, {std::numeric_limits<float>::quiet_NaN()}};
    const ValidityMask forbiddenMask{1, 1, {false}};
    const AssignmentResult result = solve(forbiddenNonFinite, forbiddenMask);
    expectCompleteCoverage(result, forbiddenNonFinite, forbiddenMask);
    EXPECT_TRUE(result.matches.empty());
}

}  // namespace
