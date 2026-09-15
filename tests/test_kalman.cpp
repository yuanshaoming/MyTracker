#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "geometry.h"
#include "kalman_box_tracker.h"

namespace {

using tracking::BBox;
using tracking::kalman::KalmanBoxTracker;

float centerX(const BBox& box) {
    const auto point = tracking::geometry::center(box);
    EXPECT_TRUE(point.has_value());
    return point ? point->x : 0.0F;
}

TEST(KalmanBoxTrackerTest, InitializationRoundTripsBox) {
    const BBox initial{-2.0F, 3.0F, 4.0F, 7.0F};
    const KalmanBoxTracker tracker(initial);

    const auto estimate = tracker.estimate();
    ASSERT_TRUE(estimate.has_value());
    EXPECT_NEAR(estimate->x1, initial.x1, 1.0e-5F);
    EXPECT_NEAR(estimate->y1, initial.y1, 1.0e-5F);
    EXPECT_NEAR(estimate->x2, initial.x2, 1.0e-5F);
    EXPECT_NEAR(estimate->y2, initial.y2, 1.0e-5F);
}

TEST(KalmanBoxTrackerTest, PredictsConstantVelocityAfterMeasurement) {
    KalmanBoxTracker tracker(BBox{0.0F, 0.0F, 2.0F, 2.0F});
    ASSERT_TRUE(tracker.predict());
    ASSERT_TRUE(tracker.update(BBox{2.0F, 0.0F, 4.0F, 2.0F}));

    const auto afterUpdate = tracker.estimate();
    ASSERT_TRUE(afterUpdate.has_value());
    EXPECT_NEAR(centerX(*afterUpdate), 2.9821429F, 1.0e-4F);

    ASSERT_TRUE(tracker.predict());
    const auto afterPrediction = tracker.estimate();
    ASSERT_TRUE(afterPrediction.has_value());
    EXPECT_NEAR(centerX(*afterPrediction), 4.7678571F, 1.0e-4F);
}

TEST(KalmanBoxTrackerTest, CorrectsAfterStoppingThenMovingAgain) {
    KalmanBoxTracker tracker(BBox{0.0F, 0.0F, 2.0F, 2.0F});
    ASSERT_TRUE(tracker.predict());
    ASSERT_TRUE(tracker.update(BBox{2.0F, 0.0F, 4.0F, 2.0F}));
    ASSERT_TRUE(tracker.predict());
    ASSERT_TRUE(tracker.update(BBox{2.0F, 0.0F, 4.0F, 2.0F}));

    const auto stationaryEstimate = tracker.estimate();
    ASSERT_TRUE(stationaryEstimate.has_value());
    EXPECT_NEAR(centerX(*stationaryEstimate), 3.0F, 0.25F);

    ASSERT_TRUE(tracker.predict());
    ASSERT_TRUE(tracker.update(BBox{4.0F, 0.0F, 6.0F, 2.0F}));
    const auto movedEstimate = tracker.estimate();
    ASSERT_TRUE(movedEstimate.has_value());
    EXPECT_GT(centerX(*movedEstimate), centerX(*stationaryEstimate) + 0.5F);
    EXPECT_NEAR(centerX(*movedEstimate), 5.0F, 0.25F);
}

TEST(KalmanBoxTrackerTest, RemainsFiniteAcrossPredictionsAndReobserves) {
    KalmanBoxTracker tracker(BBox{0.0F, 0.0F, 2.0F, 2.0F});
    for (int prediction = 0; prediction < 5; ++prediction) {
        ASSERT_TRUE(tracker.predict());
        const auto estimate = tracker.estimate();
        ASSERT_TRUE(estimate.has_value());
        EXPECT_TRUE(tracking::geometry::isValid(*estimate));
        EXPECT_TRUE(std::isfinite(estimate->x1));
        EXPECT_TRUE(std::isfinite(estimate->y1));
        EXPECT_TRUE(std::isfinite(estimate->x2));
        EXPECT_TRUE(std::isfinite(estimate->y2));
    }

    ASSERT_TRUE(tracker.update(BBox{8.0F, 0.0F, 10.0F, 2.0F}));
    const auto reobserved = tracker.estimate();
    ASSERT_TRUE(reobserved.has_value());
    EXPECT_NEAR(centerX(*reobserved), 9.0F, 0.1F);
}

TEST(KalmanBoxTrackerTest, UpdatesSizeAndAspectRatio) {
    KalmanBoxTracker tracker(BBox{0.0F, 0.0F, 2.0F, 2.0F});
    ASSERT_TRUE(tracker.predict());
    ASSERT_TRUE(tracker.update(BBox{0.0F, 0.0F, 4.0F, 2.0F}));

    const auto estimate = tracker.estimate();
    ASSERT_TRUE(estimate.has_value());
    const auto observation = tracking::geometry::toObservation(*estimate);
    ASSERT_TRUE(observation.has_value());
    EXPECT_NEAR(observation->area, 7.9642857F, 1.0e-4F);
    EXPECT_NEAR(observation->aspectRatio, 1.9166667F, 1.0e-4F);
}

TEST(KalmanBoxTrackerTest, RejectsInvalidObservationsWithoutChangingState) {
    KalmanBoxTracker tracker(BBox{0.0F, 0.0F, 2.0F, 2.0F});
    const auto before = tracker.estimate();
    ASSERT_TRUE(before.has_value());

    EXPECT_FALSE(tracker.update(BBox{0.0F, 0.0F, 0.0F, 2.0F}));
    EXPECT_FALSE(tracker.update(BBox{
        0.0F,
        0.0F,
        std::numeric_limits<float>::quiet_NaN(),
        2.0F}));
    const auto after = tracker.estimate();
    ASSERT_TRUE(after.has_value());
    EXPECT_FLOAT_EQ(after->x1, before->x1);
    EXPECT_FLOAT_EQ(after->y1, before->y1);
    EXPECT_FLOAT_EQ(after->x2, before->x2);
    EXPECT_FLOAT_EQ(after->y2, before->y2);

    EXPECT_THROW(
        KalmanBoxTracker(BBox{0.0F, 0.0F, 0.0F, 2.0F}),
        std::invalid_argument);
}

TEST(KalmanBoxTrackerTest, FailsWhenPredictedAreaBecomesNonPositive) {
    KalmanBoxTracker tracker(BBox{0.0F, 0.0F, 10.0F, 10.0F});
    ASSERT_TRUE(tracker.predict());
    ASSERT_TRUE(tracker.update(BBox{0.0F, 0.0F, 1.0F, 1.0F}));

    EXPECT_FALSE(tracker.predict());
    EXPECT_FALSE(tracker.estimate().has_value());
    EXPECT_FALSE(tracker.update(BBox{0.0F, 0.0F, 2.0F, 2.0F}));
}

}  // namespace
