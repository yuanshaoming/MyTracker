#pragma once

namespace tracking {

struct BBox {
    float x1 = 0.0F;
    float y1 = 0.0F;
    float x2 = 0.0F;
    float y2 = 0.0F;
};

struct Detection {
    BBox bbox{};
    float confidence = 1.0F;
    int classId = 0;
};

enum class TrackState {
    Tentative,
    Confirmed,
    Lost,
};

struct TrackResult {
    int trackId = -1;
    BBox bbox{};
    float confidence = 0.0F;
    TrackState state = TrackState::Tentative;
    float velocityX = 0.0F;
    float velocityY = 0.0F;
    int age = 0;
    int lostFrames = 0;
};

}  // namespace tracking
