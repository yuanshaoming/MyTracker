#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "tracking/sort_tracker.h"

namespace {

using tracking::BBox;
using tracking::Detection;
using tracking::SortTracker;
using tracking::TrackResult;
using tracking::TrackState;
using tracking::TrackerConfig;

Detection detection(float x1, float y1, float x2, float y2, float confidence = 0.9F) {
    return Detection{BBox{x1, y1, x2, y2}, confidence, 0};
}

const TrackResult& trackWithId(const std::vector<TrackResult>& results, int trackId) {
    for (const TrackResult& result : results) {
        if (result.trackId == trackId) {
            return result;
        }
    }
    ADD_FAILURE() << "missing track " << trackId;
    static const TrackResult missing;
    return missing;
}

void expectEqualResults(const std::vector<TrackResult>& left, const std::vector<TrackResult>& right) {
    ASSERT_EQ(left.size(), right.size());
    for (std::size_t index = 0; index < left.size(); ++index) {
        EXPECT_EQ(left[index].trackId, right[index].trackId);
        EXPECT_NEAR(left[index].bbox.x1, right[index].bbox.x1, 1.0e-5F);
        EXPECT_NEAR(left[index].bbox.y1, right[index].bbox.y1, 1.0e-5F);
        EXPECT_NEAR(left[index].bbox.x2, right[index].bbox.x2, 1.0e-5F);
        EXPECT_NEAR(left[index].bbox.y2, right[index].bbox.y2, 1.0e-5F);
        EXPECT_NEAR(left[index].confidence, right[index].confidence, 1.0e-5F);
        EXPECT_EQ(left[index].state, right[index].state);
        EXPECT_NEAR(left[index].velocityX, right[index].velocityX, 1.0e-5F);
        EXPECT_NEAR(left[index].velocityY, right[index].velocityY, 1.0e-5F);
        EXPECT_EQ(left[index].age, right[index].age);
        EXPECT_EQ(left[index].lostFrames, right[index].lostFrames);
    }
}

TEST(SortTrackerTest, KeepsSinglePersonIdAndConfirmsAtMinHits) {
    SortTracker tracker;

    const auto first = tracker.update({detection(0.0F, 0.0F, 4.0F, 2.0F)}, 0);
    ASSERT_EQ(first.size(), 1U);
    EXPECT_EQ(first.front().trackId, 1);
    EXPECT_EQ(first.front().state, TrackState::Tentative);
    EXPECT_EQ(first.front().age, 1);

    const auto second = tracker.update({detection(1.0F, 0.0F, 5.0F, 2.0F)}, 125);
    ASSERT_EQ(second.size(), 1U);
    EXPECT_EQ(second.front().trackId, 1);
    EXPECT_EQ(second.front().state, TrackState::Confirmed);
    EXPECT_EQ(second.front().age, 2);
    EXPECT_EQ(second.front().lostFrames, 0);
    EXPECT_GT(second.front().velocityX, 0.0F);

    const auto third = tracker.update({detection(2.0F, 0.0F, 6.0F, 2.0F)}, 250);
    ASSERT_EQ(third.size(), 1U);
    EXPECT_EQ(third.front().trackId, 1);
    EXPECT_EQ(third.front().state, TrackState::Confirmed);
    EXPECT_EQ(third.front().age, 3);
}

TEST(SortTrackerTest, AssignsIndependentIdsToFarApartPeople) {
    SortTracker tracker;

    const auto first = tracker.update(
        {detection(0.0F, 0.0F, 4.0F, 2.0F), detection(100.0F, 0.0F, 104.0F, 2.0F)},
        0);
    ASSERT_EQ(first.size(), 2U);
    EXPECT_EQ(first[0].trackId, 1);
    EXPECT_EQ(first[1].trackId, 2);

    const auto second = tracker.update(
        {detection(101.0F, 0.0F, 105.0F, 2.0F), detection(1.0F, 0.0F, 5.0F, 2.0F)},
        125);
    ASSERT_EQ(second.size(), 2U);
    EXPECT_EQ(second[0].trackId, 1);
    EXPECT_EQ(second[1].trackId, 2);
    EXPECT_EQ(trackWithId(second, 1).state, TrackState::Confirmed);
    EXPECT_EQ(trackWithId(second, 2).state, TrackState::Confirmed);
}

TEST(SortTrackerTest, HandlesPersonEnteringAndLeavingTheStream) {
    SortTracker tracker;
    tracker.update({detection(0.0F, 0.0F, 4.0F, 2.0F)}, 0);
    tracker.update({detection(1.0F, 0.0F, 5.0F, 2.0F)}, 125);

    const auto entered = tracker.update(
        {detection(2.0F, 0.0F, 6.0F, 2.0F), detection(100.0F, 0.0F, 104.0F, 2.0F)},
        250);
    ASSERT_EQ(entered.size(), 2U);
    EXPECT_EQ(trackWithId(entered, 1).state, TrackState::Confirmed);
    EXPECT_EQ(trackWithId(entered, 2).state, TrackState::Tentative);

    const auto confirmed = tracker.update(
        {detection(3.0F, 0.0F, 7.0F, 2.0F), detection(100.5F, 0.0F, 104.5F, 2.0F)},
        375);
    ASSERT_EQ(confirmed.size(), 2U);
    EXPECT_EQ(trackWithId(confirmed, 2).state, TrackState::Confirmed);

    const auto left = tracker.update({detection(4.0F, 0.0F, 8.0F, 2.0F)}, 500);
    ASSERT_EQ(left.size(), 2U);
    EXPECT_EQ(trackWithId(left, 1).state, TrackState::Confirmed);
    EXPECT_EQ(trackWithId(left, 2).state, TrackState::Lost);
}

TEST(SortTrackerTest, DeletesTentativeTrackOnFirstMiss) {
    SortTracker tracker;
    ASSERT_EQ(tracker.update({detection(0.0F, 0.0F, 2.0F, 2.0F)}, 0).size(), 1U);
    EXPECT_TRUE(tracker.update({}, 125).empty());
}

TEST(SortTrackerTest, HandlesLostDeletionBoundaryAndRecovery) {
    SortTracker tracker;
    ASSERT_EQ(tracker.update({detection(0.0F, 0.0F, 2.0F, 2.0F)}, 0).size(), 1U);
    ASSERT_EQ(tracker.update({detection(0.0F, 0.0F, 2.0F, 2.0F)}, 125).front().state,
              TrackState::Confirmed);

    const auto lost = tracker.update({}, 250);
    ASSERT_EQ(lost.size(), 1U);
    EXPECT_EQ(lost.front().trackId, 1);
    EXPECT_EQ(lost.front().state, TrackState::Lost);
    EXPECT_EQ(lost.front().lostFrames, 1);

    const auto recovered = tracker.update({detection(0.0F, 0.0F, 2.0F, 2.0F)}, 375);
    ASSERT_EQ(recovered.size(), 1U);
    EXPECT_EQ(recovered.front().trackId, 1);
    EXPECT_EQ(recovered.front().state, TrackState::Confirmed);
    EXPECT_EQ(recovered.front().lostFrames, 0);

    for (int miss = 1; miss <= 8; ++miss) {
        const auto results = tracker.update({}, 375 + miss * 125);
        ASSERT_EQ(results.size(), 1U);
        EXPECT_EQ(results.front().state, TrackState::Lost);
        EXPECT_EQ(results.front().lostFrames, miss);
    }
    EXPECT_TRUE(tracker.update({}, 1500).empty());
}

TEST(SortTrackerTest, TreatsEmptyAndInvalidDetectionsEquivalently) {
    SortTracker emptyTracker;
    SortTracker invalidTracker;
    const std::vector<Detection> valid{detection(0.0F, 0.0F, 2.0F, 2.0F)};
    emptyTracker.update(valid, 0);
    emptyTracker.update(valid, 125);
    invalidTracker.update(valid, 0);
    invalidTracker.update(valid, 125);

    const std::vector<Detection> invalid{
        detection(0.0F, 0.0F, 0.0F, 2.0F),
        detection(0.0F, 0.0F, 2.0F, 2.0F, 0.4F),
        detection(0.0F, 0.0F, 2.0F, 2.0F, std::numeric_limits<float>::quiet_NaN()),
    };
    expectEqualResults(
        emptyTracker.update({}, 250),
        invalidTracker.update(invalid, 250));
}

TEST(SortTrackerTest, ResetMatchesNewInstanceAndInstancesAreIndependent) {
    const std::vector<Detection> input{detection(0.0F, 0.0F, 2.0F, 2.0F)};
    SortTracker resetTracker;
    resetTracker.update(input, 0);
    resetTracker.reset();

    SortTracker freshTracker;
    expectEqualResults(resetTracker.update(input, 0), freshTracker.update(input, 0));

    SortTracker first;
    SortTracker second;
    const auto firstResults = first.update(input, 0);
    const auto secondResults = second.update(input, 0);
    ASSERT_EQ(firstResults.size(), 1U);
    ASSERT_EQ(secondResults.size(), 1U);
    EXPECT_EQ(firstResults.front().trackId, 1);
    EXPECT_EQ(secondResults.front().trackId, 1);
}

TEST(SortTrackerTest, TimestampFailuresDoNotChangeStateAndIntervalsKeepFixedDt) {
    const std::vector<Detection> firstDetection{detection(0.0F, 0.0F, 4.0F, 2.0F)};
    const std::vector<Detection> secondDetection{detection(1.0F, 0.0F, 5.0F, 2.0F)};
    SortTracker failedTimestampTracker;
    SortTracker controlTracker;
    failedTimestampTracker.update(firstDetection, 0);
    controlTracker.update(firstDetection, 0);
    EXPECT_THROW(failedTimestampTracker.update(secondDetection, 0), std::invalid_argument);
    EXPECT_THROW(failedTimestampTracker.update(secondDetection, -1), std::invalid_argument);
    expectEqualResults(
        failedTimestampTracker.update(secondDetection, 125),
        controlTracker.update(secondDetection, 125));

    SortTracker regularTracker;
    SortTracker irregularTracker;
    regularTracker.update(firstDetection, 0);
    irregularTracker.update(firstDetection, 0);
    regularTracker.update(secondDetection, 125);
    irregularTracker.update(secondDetection, 10);
    expectEqualResults(
        regularTracker.update({detection(2.0F, 0.0F, 6.0F, 2.0F)}, 250),
        irregularTracker.update({detection(2.0F, 0.0F, 6.0F, 2.0F)}, 1000));
}

TEST(SortTrackerTest, RejectsInvalidConfiguration) {
    TrackerConfig invalidConfidence;
    invalidConfidence.detectionThreshold = 1.1F;
    EXPECT_THROW({ SortTracker tracker(invalidConfidence); }, std::invalid_argument);

    TrackerConfig invalidIoU;
    invalidIoU.iouThreshold = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW({ SortTracker tracker(invalidIoU); }, std::invalid_argument);

    TrackerConfig invalidHits;
    invalidHits.minHits = 0;
    EXPECT_THROW({ SortTracker tracker(invalidHits); }, std::invalid_argument);

    TrackerConfig invalidFps;
    invalidFps.nominalFps = 0.0F;
    EXPECT_THROW({ SortTracker tracker(invalidFps); }, std::invalid_argument);
}

}  // namespace
