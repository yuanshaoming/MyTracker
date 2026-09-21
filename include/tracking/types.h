#pragma once

namespace tracking {

struct BBox {
    BBox() = default;
    BBox(float left, float top, float right, float bottom)
        : x1(left), y1(top), x2(right), y2(bottom) {}

    float x1 = 0.0F;
    float y1 = 0.0F;
    float x2 = 0.0F;
    float y2 = 0.0F;
};

struct Detection {
    Detection() = default;
    Detection(BBox box, float detectionConfidence, int detectionClassId)
        : bbox(box), confidence(detectionConfidence), classId(detectionClassId) {}

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
    TrackResult() = default;
    TrackResult(
        int id,
        BBox box,
        float detectionConfidence,
        TrackState trackState,
        float vx,
        float vy,
        int trackAge,
        int missedFrames)
        : trackId(id),
          bbox(box),
          confidence(detectionConfidence),
          state(trackState),
          velocityX(vx),
          velocityY(vy),
          age(trackAge),
          lostFrames(missedFrames) {}

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
