#include <cstdint>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "tracking/itracker.h"
#include "tracking/ocsort_tracker.h"
#include "tracking/sort_tracker.h"
#include "tracking/tracker_config.h"
#include "tracking/types.h"

namespace {

using UpdateSignature = std::vector<tracking::TrackResult> (tracking::ITracker::*)(
    const std::vector<tracking::Detection>&,
    std::int64_t);
using ResetSignature = void (tracking::ITracker::*)();

static_assert(std::is_abstract<tracking::ITracker>::value, "ITracker must remain abstract");
static_assert(
    std::has_virtual_destructor<tracking::ITracker>::value,
    "ITracker must retain a virtual destructor");
static_assert(
    std::is_base_of<tracking::ITracker, tracking::SortTracker>::value,
    "SortTracker must implement ITracker");
static_assert(
    std::is_base_of<tracking::ITracker, tracking::OCSortTracker>::value,
    "OCSortTracker must implement ITracker");
static_assert(
    std::is_same<decltype(&tracking::ITracker::update), UpdateSignature>::value,
    "ITracker::update signature changed");
static_assert(
    std::is_same<decltype(&tracking::ITracker::reset), ResetSignature>::value,
    "ITracker::reset signature changed");
static_assert(
    std::is_constructible<tracking::BBox, float, float, float, float>::value,
    "BBox must support positional construction on VS2015");
static_assert(
    std::is_constructible<tracking::Detection, tracking::BBox, float, int>::value,
    "Detection must support positional construction on VS2015");
static_assert(
    std::is_constructible<
        tracking::TrackResult,
        int,
        tracking::BBox,
        float,
        tracking::TrackState,
        float,
        float,
        int,
        int>::value,
    "TrackResult must support positional construction on VS2015");

}  // namespace

TEST(PublicTypesTest, DefaultsAreDeterministic) {
    const tracking::BBox bbox;
    EXPECT_FLOAT_EQ(bbox.x1, 0.0F);
    EXPECT_FLOAT_EQ(bbox.y1, 0.0F);
    EXPECT_FLOAT_EQ(bbox.x2, 0.0F);
    EXPECT_FLOAT_EQ(bbox.y2, 0.0F);

    const tracking::Detection detection;
    EXPECT_FLOAT_EQ(detection.bbox.x1, 0.0F);
    EXPECT_FLOAT_EQ(detection.confidence, 1.0F);
    EXPECT_EQ(detection.classId, 0);

    const tracking::TrackResult result;
    EXPECT_EQ(result.trackId, -1);
    EXPECT_FLOAT_EQ(result.bbox.x1, 0.0F);
    EXPECT_FLOAT_EQ(result.confidence, 0.0F);
    EXPECT_EQ(result.state, tracking::TrackState::Tentative);
    EXPECT_FLOAT_EQ(result.velocityX, 0.0F);
    EXPECT_FLOAT_EQ(result.velocityY, 0.0F);
    EXPECT_EQ(result.age, 0);
    EXPECT_EQ(result.lostFrames, 0);
}

TEST(TrackerConfigTest, DefaultsMatchContract) {
    const tracking::TrackerConfig config;
    EXPECT_FLOAT_EQ(config.detectionThreshold, 0.5F);
    EXPECT_FLOAT_EQ(config.continuationDetectionThreshold, 0.4F);
    EXPECT_FLOAT_EQ(config.iouThreshold, 0.3F);
    EXPECT_EQ(config.minHits, 2);
    EXPECT_EQ(config.maxAgeFrames, 2);
    EXPECT_FLOAT_EQ(config.nominalFps, 8.0F);
    EXPECT_EQ(config.deltaT, 3);
    EXPECT_FLOAT_EQ(config.inertia, 0.2F);
}

TEST(ITrackerTest, InterfaceSignatureCompiles) {
    SUCCEED();
}
