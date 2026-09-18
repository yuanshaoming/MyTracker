#include "ocsort_reupdate.h"

#include <cmath>

#include "geometry.h"

namespace tracking::ocsort {
namespace {

std::optional<BBox> interpolate(
    const BBox& first,
    const BBox& second,
    float ratio) {
    const auto firstCenter = geometry::center(first);
    const auto secondCenter = geometry::center(second);
    if (!firstCenter || !secondCenter || !std::isfinite(ratio) || ratio <= 0.0F || ratio >= 1.0F) {
        return std::nullopt;
    }

    const float firstWidth = first.x2 - first.x1;
    const float firstHeight = first.y2 - first.y1;
    const float secondWidth = second.x2 - second.x1;
    const float secondHeight = second.y2 - second.y1;
    const float centerX = firstCenter->x + (secondCenter->x - firstCenter->x) * ratio;
    const float centerY = firstCenter->y + (secondCenter->y - firstCenter->y) * ratio;
    const float width = firstWidth + (secondWidth - firstWidth) * ratio;
    const float height = firstHeight + (secondHeight - firstHeight) * ratio;
    const BBox result{
        centerX - width * 0.5F,
        centerY - height * 0.5F,
        centerX + width * 0.5F,
        centerY + height * 0.5F,
    };
    return geometry::isValid(result) ? std::optional<BBox>{result} : std::nullopt;
}

}  // namespace

std::optional<ReupdateResult> reupdate(
    kalman::KalmanBoxTracker& filter,
    const kalman::KalmanBoxTracker& posterior,
    const BBox& lastObservation,
    int lastObservationAge,
    const BBox& currentObservation,
    int currentAge) {
    if (!geometry::isValid(lastObservation) || !geometry::isValid(currentObservation) ||
        lastObservationAge < 1 || currentAge <= lastObservationAge + 1) {
        return std::nullopt;
    }

    kalman::KalmanBoxTracker replayed = posterior;
    ReupdateResult result;
    const int span = currentAge - lastObservationAge;
    for (int step = 1; step <= span; ++step) {
        std::optional<BBox> interpolated;
        const BBox* observation = &currentObservation;
        if (step < span) {
            interpolated = interpolate(
                lastObservation,
                currentObservation,
                static_cast<float>(step) / static_cast<float>(span));
            if (!interpolated) {
                return std::nullopt;
            }
            observation = &*interpolated;
            result.virtualObservations.push_back(*observation);
        }
        if (!replayed.predict() || !replayed.update(*observation)) {
            return std::nullopt;
        }
        if (step < span) {
            ++result.virtualObservationCount;
        }
    }
    filter = replayed;
    return result;
}

}  // namespace tracking::ocsort
