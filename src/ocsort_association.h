#pragma once

#include <optional>
#include <vector>

#include "hungarian.h"
#include "tracking/types.h"

namespace tracking::ocsort {

constexpr int kDefaultDeltaT = 3;
constexpr float kDefaultInertia = 0.2F;

struct Observation {
    BBox bbox{};
    int age = 0;
};

class ObservationHistory {
public:
    void record(const Detection& detection, int age);
    std::optional<Observation> latestBefore(int currentAge) const noexcept;
    std::optional<Observation> priorTo(int currentAge, int deltaT) const noexcept;
    void reset() noexcept;

private:
    std::vector<Observation> observations_;
};

struct CandidateTrack {
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
std::optional<float> similarity(
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

}  // namespace tracking::ocsort
