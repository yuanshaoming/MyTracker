#pragma once

namespace tracking {

struct TrackerConfig {
    float detectionThreshold = 0.5F;
    float iouThreshold = 0.3F;
    int minHits = 2;
    int maxAgeFrames = 8;
    float nominalFps = 8.0F;
};

}  // namespace tracking
