#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "geometry.h"

namespace {

using tracking::BBox;
using tracking::geometry::BBoxObservation;

TEST(GeometryTest, ValidityAreaAndCenterHandleFiniteBounds) {
    const BBox valid{-4.0F, -3.0F, 2.0F, 5.0F};
    EXPECT_TRUE(tracking::geometry::isValid(valid));
    EXPECT_FLOAT_EQ(tracking::geometry::area(valid), 48.0F);

    const auto validCenter = tracking::geometry::center(valid);
    ASSERT_TRUE(validCenter.has_value());
    EXPECT_FLOAT_EQ(validCenter->x, -1.0F);
    EXPECT_FLOAT_EQ(validCenter->y, 1.0F);

    const BBox zeroWidth{0.0F, 0.0F, 0.0F, 2.0F};
    const BBox reversed{3.0F, 0.0F, 1.0F, 2.0F};
    const BBox nonFinite{0.0F, 0.0F, std::numeric_limits<float>::infinity(), 2.0F};
    EXPECT_FALSE(tracking::geometry::isValid(zeroWidth));
    EXPECT_FALSE(tracking::geometry::isValid(reversed));
    EXPECT_FALSE(tracking::geometry::isValid(nonFinite));
    EXPECT_FLOAT_EQ(tracking::geometry::area(zeroWidth), 0.0F);
    EXPECT_FLOAT_EQ(tracking::geometry::area(reversed), 0.0F);
    EXPECT_FLOAT_EQ(tracking::geometry::area(nonFinite), 0.0F);
    EXPECT_FALSE(tracking::geometry::center(nonFinite).has_value());
}

TEST(GeometryTest, IouCoversExactOverlapAndBoundaries) {
    const BBox first{0.0F, 0.0F, 4.0F, 4.0F};
    const BBox partialOverlap{2.0F, 2.0F, 6.0F, 6.0F};
    const BBox touching{4.0F, 0.0F, 6.0F, 2.0F};
    const BBox disjoint{8.0F, 0.0F, 10.0F, 2.0F};
    const BBox contained{1.0F, 1.0F, 3.0F, 3.0F};
    const BBox negative{-2.0F, -2.0F, 2.0F, 2.0F};

    EXPECT_FLOAT_EQ(tracking::geometry::iou(first, first), 1.0F);
    EXPECT_NEAR(tracking::geometry::iou(first, partialOverlap), 1.0F / 7.0F, 1.0e-6F);
    EXPECT_FLOAT_EQ(tracking::geometry::iou(first, touching), 0.0F);
    EXPECT_FLOAT_EQ(tracking::geometry::iou(first, disjoint), 0.0F);
    EXPECT_NEAR(tracking::geometry::iou(first, contained), 0.25F, 1.0e-6F);
    EXPECT_NEAR(tracking::geometry::iou(first, negative), 1.0F / 7.0F, 1.0e-6F);
    EXPECT_FLOAT_EQ(
        tracking::geometry::iou(first, BBox{0.0F, 0.0F, 0.0F, 1.0F}),
        0.0F);
}

TEST(GeometryTest, CenterDistanceAndDirectionHandleNoMovement) {
    const BBox from{0.0F, 0.0F, 2.0F, 2.0F};
    const BBox to{3.0F, 4.0F, 5.0F, 6.0F};

    const auto distance = tracking::geometry::centerDistance(from, to);
    ASSERT_TRUE(distance.has_value());
    EXPECT_FLOAT_EQ(*distance, 5.0F);

    const auto direction = tracking::geometry::unitDirection(from, to);
    ASSERT_TRUE(direction.has_value());
    EXPECT_FLOAT_EQ(direction->x, 0.6F);
    EXPECT_FLOAT_EQ(direction->y, 0.8F);

    const auto noMovement = tracking::geometry::unitDirection(from, from);
    ASSERT_TRUE(noMovement.has_value());
    EXPECT_FLOAT_EQ(noMovement->x, 0.0F);
    EXPECT_FLOAT_EQ(noMovement->y, 0.0F);

    EXPECT_FALSE(
        tracking::geometry::centerDistance(from, BBox{0.0F, 0.0F, 0.0F, 1.0F}).has_value());
    EXPECT_FALSE(
        tracking::geometry::unitDirection(from, BBox{0.0F, 0.0F, 0.0F, 1.0F}).has_value());
}

TEST(GeometryTest, ObservationConversionRoundTripsAndRejectsInvalidInputs) {
    const BBox original{-2.0F, 3.0F, 4.0F, 7.0F};
    const auto observation = tracking::geometry::toObservation(original);
    ASSERT_TRUE(observation.has_value());
    EXPECT_FLOAT_EQ(observation->centerX, 1.0F);
    EXPECT_FLOAT_EQ(observation->centerY, 5.0F);
    EXPECT_FLOAT_EQ(observation->area, 24.0F);
    EXPECT_FLOAT_EQ(observation->aspectRatio, 1.5F);

    const auto roundTrip = tracking::geometry::fromObservation(*observation);
    ASSERT_TRUE(roundTrip.has_value());
    EXPECT_NEAR(roundTrip->x1, original.x1, 1.0e-5F);
    EXPECT_NEAR(roundTrip->y1, original.y1, 1.0e-5F);
    EXPECT_NEAR(roundTrip->x2, original.x2, 1.0e-5F);
    EXPECT_NEAR(roundTrip->y2, original.y2, 1.0e-5F);
    EXPECT_TRUE(std::isfinite(roundTrip->x1));
    EXPECT_TRUE(std::isfinite(roundTrip->y1));
    EXPECT_TRUE(std::isfinite(roundTrip->x2));
    EXPECT_TRUE(std::isfinite(roundTrip->y2));

    EXPECT_FALSE(tracking::geometry::toObservation(BBox{0.0F, 0.0F, 0.0F, 1.0F}).has_value());
    EXPECT_FALSE(tracking::geometry::fromObservation(BBoxObservation{0.0F, 0.0F, 0.0F, 1.0F}).has_value());
    EXPECT_FALSE(tracking::geometry::fromObservation(BBoxObservation{0.0F, 0.0F, 1.0F, -1.0F}).has_value());
    EXPECT_FALSE(tracking::geometry::fromObservation(BBoxObservation{
        0.0F,
        0.0F,
        std::numeric_limits<float>::quiet_NaN(),
        1.0F}).has_value());
}

}  // namespace
