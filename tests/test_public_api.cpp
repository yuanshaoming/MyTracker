#include <cstdint>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "tracking/itracker.h"
#include "tracking/sort_tracker.h"
#include "tracking/tracker_config.h"
#include "tracking/types.h"

namespace {

using UpdateSignature = std::vector<tracking::TrackResult> (tracking::ITracker::*)(
    const std::vector<tracking::Detection>&,
    std::int64_t);
using ResetSignature = void (tracking::ITracker::*)();

static_assert(std::is_abstract_v<tracking::ITracker>);
static_assert(std::has_virtual_destructor_v<tracking::ITracker>);
static_assert(std::is_base_of_v<tracking::ITracker, tracking::SortTracker>);
static_assert(std::is_same_v<decltype(&tracking::ITracker::update), UpdateSignature>);
static_assert(std::is_same_v<decltype(&tracking::ITracker::reset), ResetSignature>);

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
    EXPECT_FLOAT_EQ(config.iouThreshold, 0.3F);
    EXPECT_EQ(config.minHits, 2);
    EXPECT_EQ(config.maxAgeFrames, 8);
    EXPECT_FLOAT_EQ(config.nominalFps, 8.0F);
}

TEST(ITrackerTest, InterfaceSignatureCompiles) {
    SUCCEED();
}
