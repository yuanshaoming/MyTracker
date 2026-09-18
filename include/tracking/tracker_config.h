#pragma once

namespace tracking {

struct TrackerConfig {
    float detectionThreshold = 0.5F;
    float iouThreshold = 0.3F;
    int minHits = 2;
    int maxAgeFrames = 2;
    float nominalFps = 8.0F;
    int deltaT = 3;
    float inertia = 0.2F;
    float continuationDetectionThreshold = 0.4F;
};

}  // namespace tracking
