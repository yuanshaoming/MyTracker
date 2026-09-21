#pragma once

#include <vector>

#include "compat_optional.h"
#include "hungarian.h"
#include "tracking/types.h"

namespace tracking {
namespace ocsort {

constexpr int kDefaultDeltaT = 3;
constexpr float kDefaultInertia = 0.2F;

struct Observation {
    Observation() = default;
    Observation(BBox box, int observationAge) : bbox(box), age(observationAge) {}

    BBox bbox{};
    int age = 0;
};

class ObservationHistory {
public:
    void record(const Detection& detection, int age);
    detail::Optional<Observation> latestBefore(int currentAge) const noexcept;
    detail::Optional<Observation> priorTo(int currentAge, int deltaT) const noexcept;
    void reset() noexcept;

private:
    std::vector<Observation> observations_;
};

struct CandidateTrack {
    CandidateTrack() = default;
    CandidateTrack(
        BBox box,
        const ObservationHistory* observationHistory,
        int trackAge,
        float minimumConfidence = 0.0F,
        bool accepts = true)
        : predictedBox(box),
          history(observationHistory),
          age(trackAge),
          minimumDetectionConfidence(minimumConfidence),
          acceptsDetections(accepts) {}

    BBox predictedBox{};
    const ObservationHistory* history = nullptr;
    int age = 0;
    float minimumDetectionConfidence = 0.0F;
    bool acceptsDetections = true;
};

float angleScore(
    const ObservationHistory& history,
    int currentAge,
    const BBox& detectionBox,
    int deltaT) noexcept;
detail::Optional<float> similarity(
    const CandidateTrack& track,
    const Detection& detection,
    int deltaT,
    float inertia) noexcept;
assignment::AssignmentResult associate(
    const std::vector<CandidateTrack>& tracks,
    const std::vector<Detection>& detections,
    int deltaT,
    float inertia,
    float iouThreshold);

}  // namespace ocsort
}  // namespace tracking
