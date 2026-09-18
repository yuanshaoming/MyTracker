#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "geometry.h"
#include "hungarian.h"
#include "ocsort_association.h"

namespace {

using tracking::BBox;
using tracking::Detection;
using tracking::assignment::AssignmentResult;
using tracking::ocsort::CandidateTrack;
using tracking::ocsort::ObservationHistory;

BBox box(float centerX, float centerY = 0.0F, float width = 4.0F, float height = 4.0F) {
    return BBox{
        centerX - width * 0.5F,
        centerY - height * 0.5F,
        centerX + width * 0.5F,
        centerY + height * 0.5F,
    };
}

Detection detection(float centerX, float centerY = 0.0F, float confidence = 1.0F) {
    return Detection{box(centerX, centerY), confidence, 0};
}

void expectCompleteCoverage(
    const AssignmentResult& result,
    std::size_t trackCount,
    std::size_t detectionCount) {
    std::vector<bool> seenTracks(trackCount, false);
    std::vector<bool> seenDetections(detectionCount, false);
    for (const auto& match : result.matches) {
        ASSERT_LT(match.first, trackCount);
        ASSERT_LT(match.second, detectionCount);
        EXPECT_FALSE(seenTracks[match.first]);
        EXPECT_FALSE(seenDetections[match.second]);
        seenTracks[match.first] = true;
        seenDetections[match.second] = true;
    }
    for (const std::size_t track : result.unmatchedTracks) {
        ASSERT_LT(track, trackCount);
        EXPECT_FALSE(seenTracks[track]);
        seenTracks[track] = true;
    }
    for (const std::size_t detectionIndex : result.unmatchedDetections) {
        ASSERT_LT(detectionIndex, detectionCount);
        EXPECT_FALSE(seenDetections[detectionIndex]);
        seenDetections[detectionIndex] = true;
    }
    for (const bool seen : seenTracks) {
        EXPECT_TRUE(seen);
    }
    for (const bool seen : seenDetections) {
        EXPECT_TRUE(seen);
    }
}

ObservationHistory movingRightHistory() {
    ObservationHistory history;
    history.record(detection(0.0F), 1);
    history.record(detection(2.0F), 2);
    return history;
}

TEST(ObservationHistoryTest, RetainsOnlyRecordedObservationsAndUsesDeltaWindowFallback) {
    ObservationHistory history;
    history.record(detection(0.0F), 1);
    history.record(detection(2.0F), 2);
    history.record(detection(6.0F), 6);

    const auto latest = history.latestBefore(6);
    ASSERT_TRUE(latest.has_value());
    EXPECT_EQ(latest->age, 2);

    const auto lessThanDelta = history.priorTo(3, 3);
    ASSERT_TRUE(lessThanDelta.has_value());
    EXPECT_EQ(lessThanDelta->age, 1);

    const auto exactDelta = history.priorTo(5, 3);
    ASSERT_TRUE(exactDelta.has_value());
    EXPECT_EQ(exactDelta->age, 2);

    const auto acrossMisses = history.priorTo(9, 3);
    ASSERT_TRUE(acrossMisses.has_value());
    EXPECT_EQ(acrossMisses->age, 6);
    EXPECT_FALSE(history.priorTo(10, 3).has_value());

    history.record(detection(8.0F), 6);
    const auto overwritten = history.latestBefore(7);
    ASSERT_TRUE(overwritten.has_value());
    EXPECT_EQ(overwritten->age, 6);
    const auto center = tracking::geometry::center(overwritten->bbox);
    ASSERT_TRUE(center.has_value());
    EXPECT_FLOAT_EQ(center->x, 8.0F);
}

TEST(ObservationHistoryTest, ZeroMovementResetAndSeparateTracksAreNeutralOrIsolated) {
    ObservationHistory stationary;
    stationary.record(detection(0.0F), 1);
    stationary.record(detection(0.0F), 2);
    EXPECT_FLOAT_EQ(tracking::ocsort::angleScore(stationary, 3, box(2.0F), 3), 0.0F);

    ObservationHistory first;
    ObservationHistory second;
    first.record(detection(0.0F), 1);
    first.record(detection(2.0F), 2);
    second.record(detection(10.0F), 1);
    second.record(detection(8.0F), 2);
    EXPECT_GT(tracking::ocsort::angleScore(first, 3, box(4.0F), 3), 0.0F);
    EXPECT_LT(tracking::ocsort::angleScore(second, 3, box(12.0F), 3), 0.0F);

    first.reset();
    EXPECT_FALSE(first.latestBefore(3).has_value());
    EXPECT_FLOAT_EQ(tracking::ocsort::angleScore(first, 3, box(4.0F), 3), 0.0F);
}

TEST(OcmAssociationTest, ComputesHandCalculatedDirectionAndConfidenceWeights) {
    const ObservationHistory history = movingRightHistory();
    EXPECT_NEAR(tracking::ocsort::angleScore(history, 3, box(4.0F), 3), 0.5F, 1.0e-6F);
    EXPECT_NEAR(tracking::ocsort::angleScore(history, 3, box(0.0F, 4.0F), 3), 0.0F, 1.0e-6F);
    EXPECT_NEAR(tracking::ocsort::angleScore(history, 3, box(-4.0F), 3), -0.5F, 1.0e-6F);

    const CandidateTrack track{box(4.0F), &history, 3};
    const auto weighted = tracking::ocsort::similarity(track, detection(4.0F, 0.0F, 0.6F), 3, 0.2F);
    ASSERT_TRUE(weighted.has_value());
    EXPECT_NEAR(*weighted, 1.06F, 1.0e-6F);

    const auto noInertia = tracking::ocsort::similarity(track, detection(4.0F), 3, 0.0F);
    ASSERT_TRUE(noInertia.has_value());
    EXPECT_FLOAT_EQ(*noInertia, 1.0F);
}

TEST(OcmAssociationTest, ChoosesDirectionConsistentMatchForAnAmbiguousCrossing) {
    ObservationHistory movingRight = movingRightHistory();
    ObservationHistory movingLeft;
    movingLeft.record(detection(4.0F), 1);
    movingLeft.record(detection(2.0F), 2);

    const BBox predicted{-4.0F, -2.0F, 8.0F, 2.0F};
    const std::vector<CandidateTrack> tracks{
        CandidateTrack{predicted, &movingRight, 3},
        CandidateTrack{predicted, &movingLeft, 3},
    };
    const std::vector<Detection> detections{
        Detection{BBox{-5.0F, -2.0F, 3.0F, 2.0F}, 1.0F, 0},
        Detection{BBox{1.0F, -2.0F, 9.0F, 2.0F}, 1.0F, 0},
    };

    const AssignmentResult result = tracking::ocsort::associate(tracks, detections, 3, 0.2F, 0.3F);
    expectCompleteCoverage(result, tracks.size(), detections.size());
    EXPECT_EQ(result.matches,
              (std::vector<std::pair<std::size_t, std::size_t>>{{0, 1}, {1, 0}}));
}

TEST(OcmAssociationTest, FallsBackToPureIouWithoutHistoryOrInertia) {
    const std::vector<Detection> detections{detection(0.0F), detection(3.0F)};
    ObservationHistory history = movingRightHistory();
    const std::vector<CandidateTrack> noHistory{
        CandidateTrack{box(0.5F), nullptr, 3},
        CandidateTrack{box(2.5F), nullptr, 3},
    };
    const std::vector<CandidateTrack> withHistory{
        CandidateTrack{box(0.5F), &history, 3},
        CandidateTrack{box(2.5F), &history, 3},
    };

    tracking::assignment::CostMatrix iouCosts{noHistory.size(), detections.size(), {}};
    tracking::assignment::ValidityMask iouEdges{noHistory.size(), detections.size(), {}};
    for (const CandidateTrack& track : noHistory) {
        for (const Detection& currentDetection : detections) {
            const float overlap = tracking::geometry::iou(track.predictedBox, currentDetection.bbox);
            iouCosts.values.push_back(-overlap);
            iouEdges.values.push_back(overlap >= 0.3F);
        }
    }
    const AssignmentResult expected = tracking::assignment::solve(iouCosts, iouEdges);
    const AssignmentResult withoutHistory = tracking::ocsort::associate(noHistory, detections, 3, 0.2F, 0.3F);
    const AssignmentResult withoutInertia = tracking::ocsort::associate(withHistory, detections, 3, 0.0F, 0.3F);
    EXPECT_EQ(withoutHistory.matches, expected.matches);
    EXPECT_EQ(withoutInertia.matches, expected.matches);
    expectCompleteCoverage(withoutHistory, noHistory.size(), detections.size());
    expectCompleteCoverage(withoutInertia, withHistory.size(), detections.size());
}

TEST(OcmAssociationTest, KeepsIouGateAndHandlesEmptyInvalidAndEquivalentInputs) {
    ObservationHistory history = movingRightHistory();
    const CandidateTrack track{box(0.0F), &history, 3};
    const Detection disjoint = detection(20.0F);
    const AssignmentResult gated = tracking::ocsort::associate({track}, {disjoint}, 3, 1.0F, 0.3F);
    EXPECT_TRUE(gated.matches.empty());
    EXPECT_EQ(gated.unmatchedTracks, (std::vector<std::size_t>{0}));
    EXPECT_EQ(gated.unmatchedDetections, (std::vector<std::size_t>{0}));

    const AssignmentResult noTracks = tracking::ocsort::associate({}, {detection(0.0F)}, 3, 0.2F, 0.3F);
    EXPECT_TRUE(noTracks.matches.empty());
    EXPECT_EQ(noTracks.unmatchedDetections, (std::vector<std::size_t>{0}));
    const AssignmentResult noDetections = tracking::ocsort::associate({track}, {}, 3, 0.2F, 0.3F);
    EXPECT_TRUE(noDetections.matches.empty());
    EXPECT_EQ(noDetections.unmatchedTracks, (std::vector<std::size_t>{0}));

    Detection invalid = detection(0.0F);
    invalid.confidence = std::numeric_limits<float>::quiet_NaN();
    const AssignmentResult invalidResult = tracking::ocsort::associate({track}, {invalid}, 3, 0.2F, 0.3F);
    expectCompleteCoverage(invalidResult, 1, 1);
    EXPECT_TRUE(invalidResult.matches.empty());

    const CandidateTrack invalidTrack{BBox{0.0F, 0.0F, 0.0F, 2.0F}, &history, 3};
    const AssignmentResult invalidTrackResult = tracking::ocsort::associate(
        {invalidTrack}, {detection(0.0F)}, 3, 0.2F, 0.3F);
    expectCompleteCoverage(invalidTrackResult, 1, 1);
    EXPECT_TRUE(invalidTrackResult.matches.empty());

    const AssignmentResult first = tracking::ocsort::associate(
        {CandidateTrack{box(0.0F), nullptr, 1}, CandidateTrack{box(0.0F), nullptr, 1}},
        {detection(0.0F), detection(0.0F)}, 3, 0.2F, 0.3F);
    const AssignmentResult second = tracking::ocsort::associate(
        {CandidateTrack{box(0.0F), nullptr, 1}, CandidateTrack{box(0.0F), nullptr, 1}},
        {detection(0.0F), detection(0.0F)}, 3, 0.2F, 0.3F);
    EXPECT_EQ(first.matches, second.matches);
    expectCompleteCoverage(first, 2, 2);
}

TEST(OcmAssociationTest, RejectsInvalidHistoryAndAssociationParameters) {
    ObservationHistory history;
    Detection invalidBox = detection(0.0F);
    invalidBox.bbox.x2 = invalidBox.bbox.x1;
    EXPECT_THROW(history.record(invalidBox, 1), std::invalid_argument);
    EXPECT_THROW(history.record(detection(0.0F), 0), std::invalid_argument);

    const std::vector<CandidateTrack> tracks{CandidateTrack{box(0.0F), nullptr, 1}};
    const std::vector<Detection> detections{detection(0.0F)};
    EXPECT_THROW(tracking::ocsort::associate(tracks, detections, 0, 0.2F, 0.3F), std::invalid_argument);
    EXPECT_THROW(
        tracking::ocsort::associate(
            tracks, detections, 3, std::numeric_limits<float>::infinity(), 0.3F),
        std::invalid_argument);
    EXPECT_THROW(tracking::ocsort::associate(tracks, detections, 3, 0.2F, -0.1F), std::invalid_argument);
}

}  // namespace
