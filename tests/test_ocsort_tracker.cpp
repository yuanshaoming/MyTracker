#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "geometry.h"
#include "kalman_box_tracker.h"
#include "ocsort_reupdate.h"
#include "tracking/ocsort_tracker.h"
#include "tracking/sort_tracker.h"

namespace {

using tracking::BBox;
using tracking::Detection;
using tracking::OCSortTracker;
using tracking::SortTracker;
using tracking::TrackResult;
using tracking::TrackState;
using tracking::TrackerConfig;

BBox box(float centerX, float width = 4.0F, float height = 4.0F) {
    return BBox{
        centerX - width * 0.5F,
        -height * 0.5F,
        centerX + width * 0.5F,
        height * 0.5F,
    };
}

Detection detection(float centerX, float confidence = 0.9F, float width = 4.0F) {
    return Detection{box(centerX, width), confidence, 0};
}

Detection detection(BBox bbox, float confidence) {
    return Detection{bbox, confidence, 0};
}

float centerX(const BBox& currentBox) {
    const auto center = tracking::geometry::center(currentBox);
    EXPECT_TRUE(center.has_value());
    return center ? center->x : 0.0F;
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
        EXPECT_NEAR(left[index].bbox.x2, right[index].bbox.x2, 1.0e-5F);
        EXPECT_EQ(left[index].state, right[index].state);
        EXPECT_EQ(left[index].age, right[index].age);
        EXPECT_EQ(left[index].lostFrames, right[index].lostFrames);
    }
}

TEST(OruTest, ReplaysInterpolatedObservationsFromTheSavedPosterior) {
    tracking::kalman::KalmanBoxTracker posterior(box(0.0F));
    ASSERT_TRUE(posterior.predict());
    ASSERT_TRUE(posterior.update(box(2.0F)));
    tracking::kalman::KalmanBoxTracker replayed = posterior;

    const auto result = tracking::ocsort::reupdate(
        replayed, posterior, box(2.0F), 2, box(8.0F), 5);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->virtualObservationCount, 2);
    ASSERT_EQ(result->virtualObservations.size(), 2U);
    EXPECT_NEAR(centerX(result->virtualObservations[0]), 4.0F, 1.0e-6F);
    EXPECT_NEAR(centerX(result->virtualObservations[1]), 6.0F, 1.0e-6F);

    tracking::kalman::KalmanBoxTracker expected = posterior;
    ASSERT_TRUE(expected.predict());
    ASSERT_TRUE(expected.update(box(4.0F)));
    ASSERT_TRUE(expected.predict());
    ASSERT_TRUE(expected.update(box(6.0F)));
    ASSERT_TRUE(expected.predict());
    ASSERT_TRUE(expected.update(box(8.0F)));
    const auto expectedEstimate = expected.estimate();
    const auto replayedEstimate = replayed.estimate();
    ASSERT_TRUE(expectedEstimate.has_value());
    ASSERT_TRUE(replayedEstimate.has_value());
    EXPECT_NEAR(replayedEstimate->x1, expectedEstimate->x1, 1.0e-5F);
    EXPECT_NEAR(replayedEstimate->x2, expectedEstimate->x2, 1.0e-5F);

    ASSERT_TRUE(expected.predict());
    ASSERT_TRUE(replayed.predict());
    const auto expectedNext = expected.estimate();
    const auto replayedNext = replayed.estimate();
    ASSERT_TRUE(expectedNext.has_value());
    ASSERT_TRUE(replayedNext.has_value());
    EXPECT_TRUE(tracking::geometry::isValid(*replayedNext));
    EXPECT_NEAR(replayedNext->x1, expectedNext->x1, 1.0e-5F);
    EXPECT_FALSE(tracking::ocsort::reupdate(replayed, posterior, box(2.0F), 2, box(4.0F), 3).has_value());
}

TEST(OCSortTrackerTest, RecoversOriginalIdWithOcrWhenPredictionMissesButObservationOverlaps) {
    OCSortTracker tracker;
    tracker.update({detection(0.0F)}, 0);
    ASSERT_EQ(tracker.update({detection(2.0F)}, 125).front().state, TrackState::Confirmed);
    ASSERT_EQ(tracker.update({}, 250).front().state, TrackState::Lost);

    tracking::kalman::KalmanBoxTracker predicted(box(0.0F));
    ASSERT_TRUE(predicted.predict());
    ASSERT_TRUE(predicted.update(box(2.0F)));
    ASSERT_TRUE(predicted.predict());
    ASSERT_TRUE(predicted.predict());
    const auto predictedBox = predicted.estimate();
    ASSERT_TRUE(predictedBox.has_value());
    EXPECT_LT(tracking::geometry::iou(*predictedBox, box(2.0F)), 0.3F);

    const auto recovered = tracker.update({detection(2.0F)}, 375);
    ASSERT_EQ(recovered.size(), 1U);
    EXPECT_EQ(recovered.front().trackId, 1);
    EXPECT_EQ(recovered.front().state, TrackState::Confirmed);
    EXPECT_EQ(recovered.front().lostFrames, 0);

    OCSortTracker noRecovery;
    noRecovery.update({detection(0.0F)}, 0);
    noRecovery.update({detection(2.0F)}, 125);
    noRecovery.update({}, 250);
    const auto unmatched = noRecovery.update({detection(40.0F)}, 375);
    ASSERT_EQ(unmatched.size(), 2U);
    EXPECT_EQ(trackWithId(unmatched, 1).state, TrackState::Lost);
    EXPECT_EQ(trackWithId(unmatched, 2).state, TrackState::Tentative);
}

TEST(OCSortTrackerTest, PreservesTwoIdentitiesAcrossShortOcclusionWhereSortCreatesNewTracks) {
    const std::vector<Detection> first{detection(0.0F, 0.9F, 20.0F), detection(100.0F, 0.9F, 20.0F)};
    const std::vector<Detection> second{detection(10.0F, 0.9F, 20.0F), detection(90.0F, 0.9F, 20.0F)};
    const std::vector<Detection> recovered{detection(90.0F, 0.9F, 20.0F), detection(10.0F, 0.9F, 20.0F)};
    SortTracker sort;
    OCSortTracker ocsort;

    sort.update(first, 0);
    ocsort.update(first, 0);
    sort.update(second, 125);
    ocsort.update(second, 125);
    sort.update({}, 250);
    ocsort.update({}, 250);
    sort.update({}, 375);
    ocsort.update({}, 375);
    const auto sortResult = sort.update(recovered, 500);
    const auto ocsortResult = ocsort.update(recovered, 500);

    ASSERT_EQ(sortResult.size(), 2U);
    EXPECT_EQ(trackWithId(sortResult, 3).state, TrackState::Tentative);
    EXPECT_EQ(trackWithId(sortResult, 4).state, TrackState::Tentative);

    ASSERT_EQ(ocsortResult.size(), 2U);
    EXPECT_EQ(trackWithId(ocsortResult, 1).state, TrackState::Confirmed);
    EXPECT_EQ(trackWithId(ocsortResult, 2).state, TrackState::Confirmed);
    EXPECT_LT(centerX(trackWithId(ocsortResult, 1).bbox), centerX(trackWithId(ocsortResult, 2).bbox));
}

TEST(OCSortTrackerTest, KeepsTheDepartingTrackOutOfAnEnteringPersonAtFrames79To86) {
    OCSortTracker tracker;
    const std::vector<Detection> departingObservations{
        detection(BBox{63.0F, 168.0F, 520.0F, 1320.0F}, 0.798186779F),
        detection(BBox{66.0F, 152.0F, 533.0F, 1320.0F}, 0.785966277F),
        detection(BBox{53.0F, 130.0F, 523.0F, 1316.0F}, 0.768383384F),
        detection(BBox{6.0F, 92.0F, 491.0F, 1313.0F}, 0.770462871F),
        detection(BBox{0.0F, 66.0F, 475.0F, 1316.0F}, 0.738670349F),
        detection(BBox{0.0F, 41.0F, 466.0F, 1320.0F}, 0.746139824F),
        detection(BBox{0.0F, 12.0F, 456.0F, 1319.0F}, 0.724869847F),
        detection(BBox{0.0F, 0.0F, 447.0F, 1320.0F}, 0.699253917F),
        detection(BBox{0.0F, 0.0F, 418.0F, 1319.0F}, 0.699253917F),
        detection(BBox{3.0F, 0.0F, 418.0F, 1297.0F}, 0.58604908F),
    };
    for (std::size_t index = 0; index < departingObservations.size(); ++index) {
        tracker.update({departingObservations[index]}, static_cast<std::int64_t>(index) * 128);
    }

    const auto handoffFrame = tracker.update(
        {
            detection(BBox{126.0F, 184.0F, 453.0F, 1304.0F}, 0.809276581F),
            detection(BBox{0.0F, 0.0F, 212.0F, 1310.0F}, 0.415847361F),
        },
        1280);
    ASSERT_EQ(handoffFrame.size(), 2U);
    EXPECT_FLOAT_EQ(trackWithId(handoffFrame, 1).confidence, 0.415847361F);
    EXPECT_FLOAT_EQ(trackWithId(handoffFrame, 2).confidence, 0.809276581F);
    EXPECT_EQ(trackWithId(handoffFrame, 1).state, TrackState::Confirmed);
    EXPECT_EQ(trackWithId(handoffFrame, 2).state, TrackState::Tentative);

    const auto followingFrame = tracker.update(
        {detection(BBox{72.0F, 161.0F, 495.0F, 1304.0F}, 0.830592752F)}, 1408);
    ASSERT_EQ(followingFrame.size(), 2U);
    EXPECT_EQ(trackWithId(followingFrame, 1).state, TrackState::Lost);
    EXPECT_EQ(trackWithId(followingFrame, 2).state, TrackState::Confirmed);
}

TEST(OCSortTrackerTest, DoesNotRecoverALostTrackFromAWeakContinuation) {
    OCSortTracker tracker;
    tracker.update({detection(0.0F)}, 0);
    tracker.update({detection(1.0F)}, 125);
    ASSERT_EQ(tracker.update({detection(2.0F, 0.45F)}, 250).front().state, TrackState::Confirmed);
    ASSERT_EQ(tracker.update({}, 375).front().state, TrackState::Lost);

    const auto results = tracker.update({detection(2.0F)}, 500);
    ASSERT_EQ(results.size(), 2U);
    EXPECT_EQ(trackWithId(results, 1).state, TrackState::Lost);
    EXPECT_EQ(trackWithId(results, 2).state, TrackState::Tentative);
}

TEST(OCSortTrackerTest, KeepsLifecycleInputTimestampAndResetContracts) {
    OCSortTracker tracker;
    ASSERT_EQ(tracker.update({detection(0.0F)}, 0).front().state, TrackState::Tentative);
    ASSERT_EQ(tracker.update({detection(0.0F)}, 125).front().state, TrackState::Confirmed);
    ASSERT_EQ(tracker.update({}, 250).front().lostFrames, 1);
    ASSERT_EQ(tracker.update({}, 375).front().lostFrames, 2);
    EXPECT_TRUE(tracker.update({}, 500).empty());

    OCSortTracker invalidTracker;
    OCSortTracker emptyTracker;
    invalidTracker.update({detection(0.0F)}, 0);
    emptyTracker.update({detection(0.0F)}, 0);
    invalidTracker.update({detection(0.0F)}, 125);
    emptyTracker.update({detection(0.0F)}, 125);
    Detection invalid = detection(0.0F);
    invalid.confidence = std::numeric_limits<float>::quiet_NaN();
    expectEqualResults(invalidTracker.update({invalid}, 250), emptyTracker.update({}, 250));

    OCSortTracker failedTimestamp;
    OCSortTracker control;
    failedTimestamp.update({detection(0.0F)}, 0);
    control.update({detection(0.0F)}, 0);
    EXPECT_THROW(failedTimestamp.update({detection(1.0F)}, 0), std::invalid_argument);
    EXPECT_THROW(failedTimestamp.update({detection(1.0F)}, -1), std::invalid_argument);
    expectEqualResults(
        failedTimestamp.update({detection(1.0F)}, 125),
        control.update({detection(1.0F)}, 125));

    OCSortTracker regular;
    OCSortTracker irregular;
    regular.update({detection(0.0F)}, 0);
    irregular.update({detection(0.0F)}, 0);
    regular.update({detection(1.0F)}, 125);
    irregular.update({detection(1.0F)}, 10);
    expectEqualResults(
        regular.update({detection(2.0F)}, 250),
        irregular.update({detection(2.0F)}, 1000));

    tracker.reset();
    OCSortTracker fresh;
    expectEqualResults(tracker.update({detection(0.0F)}, 0), fresh.update({detection(0.0F)}, 0));

    OCSortTracker first;
    OCSortTracker second;
    const auto firstResult = first.update({detection(0.0F)}, 0);
    const auto secondResult = second.update({detection(0.0F)}, 0);
    ASSERT_EQ(firstResult.size(), 1U);
    ASSERT_EQ(secondResult.size(), 1U);
    EXPECT_EQ(firstResult.front().trackId, 1);
    EXPECT_EQ(secondResult.front().trackId, 1);
}

TEST(OCSortTrackerTest, ValidatesOcmConfiguration) {
    TrackerConfig invalidDeltaT;
    invalidDeltaT.deltaT = 0;
    EXPECT_THROW(OCSortTracker tracker(invalidDeltaT), std::invalid_argument);
    EXPECT_THROW(SortTracker tracker(invalidDeltaT), std::invalid_argument);

    TrackerConfig invalidInertia;
    invalidInertia.inertia = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(OCSortTracker tracker(invalidInertia), std::invalid_argument);
    EXPECT_THROW(SortTracker tracker(invalidInertia), std::invalid_argument);

    TrackerConfig invalidContinuationThreshold;
    invalidContinuationThreshold.continuationDetectionThreshold =
        std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(OCSortTracker tracker(invalidContinuationThreshold), std::invalid_argument);
    EXPECT_THROW(SortTracker tracker(invalidContinuationThreshold), std::invalid_argument);
}

}  // namespace
