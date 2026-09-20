#include "ocsort_association.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "geometry.h"

namespace tracking {
namespace ocsort {
namespace {

bool isUsableDetection(const Detection& detection) noexcept {
    return geometry::isValid(detection.bbox) && std::isfinite(detection.confidence) &&
           detection.confidence >= 0.0F && detection.confidence <= 1.0F;
}

bool isZeroDirection(const geometry::Point& direction) noexcept {
    return direction.x == 0.0F && direction.y == 0.0F;
}

void validateParameters(int deltaT, float inertia, float iouThreshold) {
    if (deltaT < 1 || !std::isfinite(inertia) || inertia < 0.0F || inertia > 1.0F ||
        !std::isfinite(iouThreshold) || iouThreshold < 0.0F || iouThreshold > 1.0F) {
        throw std::invalid_argument("invalid OCM association parameters");
    }
}

}  // namespace

void ObservationHistory::record(const Detection& detection, int age) {
    if (age < 1 || !isUsableDetection(detection)) {
        throw std::invalid_argument("an observation must be a valid detection with a positive age");
    }

    const auto existing = std::find_if(
        observations_.begin(),
        observations_.end(),
        [age](const Observation& observation) { return observation.age == age; });
    if (existing != observations_.end()) {
        existing->bbox = detection.bbox;
        return;
    }

    observations_.push_back(Observation{detection.bbox, age});
}

detail::Optional<Observation> ObservationHistory::latestBefore(int currentAge) const noexcept {
    detail::Optional<Observation> latest;
    for (const Observation& observation : observations_) {
        if (observation.age < currentAge &&
            (!latest || observation.age > latest->age)) {
            latest = observation;
        }
    }
    return latest;
}

detail::Optional<Observation> ObservationHistory::priorTo(int currentAge, int deltaT) const noexcept {
    if (currentAge < 1 || deltaT < 1) {
        return {};
    }

    for (int age = currentAge - deltaT; age < currentAge; ++age) {
        if (age < 1) {
            continue;
        }
        const auto found = std::find_if(
            observations_.begin(),
            observations_.end(),
            [age](const Observation& observation) { return observation.age == age; });
        if (found != observations_.end()) {
            return *found;
        }
    }
    return {};
}

void ObservationHistory::reset() noexcept {
    observations_.clear();
}

float angleScore(
    const ObservationHistory& history,
    int currentAge,
    const BBox& detectionBox,
    int deltaT) noexcept {
    const auto prior = history.priorTo(currentAge, deltaT);
    const auto latest = history.latestBefore(currentAge);
    if (!prior || !latest || !geometry::isValid(detectionBox)) {
        return 0.0F;
    }

    const auto motion = geometry::unitDirection(prior->bbox, latest->bbox);
    const auto candidate = geometry::unitDirection(prior->bbox, detectionBox);
    if (!motion || !candidate || isZeroDirection(*motion) || isZeroDirection(*candidate)) {
        return 0.0F;
    }

    const double unboundedDot = static_cast<double>(motion->x) * candidate->x +
                                static_cast<double>(motion->y) * candidate->y;
    const double dot = std::max(-1.0, std::min(unboundedDot, 1.0));
    const double score = (std::acos(-1.0) * 0.5 - std::abs(std::acos(dot))) / std::acos(-1.0);
    return std::isfinite(score) ? static_cast<float>(score) : 0.0F;
}

detail::Optional<float> similarity(
    const CandidateTrack& track,
    const Detection& detection,
    int deltaT,
    float inertia) noexcept {
    if (deltaT < 1 || !std::isfinite(inertia) || inertia < 0.0F || inertia > 1.0F ||
        !geometry::isValid(track.predictedBox) || !isUsableDetection(detection)) {
        return {};
    }

    const float overlap = geometry::iou(track.predictedBox, detection.bbox);
    const float direction = track.history
                                ? angleScore(*track.history, track.age, detection.bbox, deltaT)
                                : 0.0F;
    const float result = overlap + inertia * detection.confidence * direction;
    return std::isfinite(result) ? detail::Optional<float>{result} : detail::Optional<float>{};
}

assignment::AssignmentResult associate(
    const std::vector<CandidateTrack>& tracks,
    const std::vector<Detection>& detections,
    int deltaT,
    float inertia,
    float iouThreshold) {
    validateParameters(deltaT, inertia, iouThreshold);

    assignment::CostMatrix costs{tracks.size(), detections.size(), {}};
    assignment::ValidityMask validEdges{tracks.size(), detections.size(), {}};
    costs.values.reserve(tracks.size() * detections.size());
    validEdges.values.reserve(tracks.size() * detections.size());

    for (const CandidateTrack& track : tracks) {
        for (const Detection& detection : detections) {
            const auto score = similarity(track, detection, deltaT, inertia);
            const float overlap = geometry::iou(track.predictedBox, detection.bbox);
            const bool valid = track.acceptsDetections && score && overlap >= iouThreshold &&
                               detection.confidence >= track.minimumDetectionConfidence;
            costs.values.push_back(valid ? -*score : 0.0F);
            validEdges.values.push_back(valid);
        }
    }
    return assignment::solve(costs, validEdges);
}

}  // namespace ocsort
}  // namespace tracking
