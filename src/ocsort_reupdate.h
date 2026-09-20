#pragma once

#include <vector>

#include "compat_optional.h"
#include "kalman_box_tracker.h"

namespace tracking {
namespace ocsort {

struct ReupdateResult {
    int virtualObservationCount = 0;
    std::vector<BBox> virtualObservations;
};

detail::Optional<ReupdateResult> reupdate(
    kalman::KalmanBoxTracker& filter,
    const kalman::KalmanBoxTracker& posterior,
    const BBox& lastObservation,
    int lastObservationAge,
    const BBox& currentObservation,
    int currentAge);

}  // namespace ocsort
}  // namespace tracking
