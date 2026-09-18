#pragma once

#include <optional>
#include <vector>

#include "kalman_box_tracker.h"

namespace tracking::ocsort {

struct ReupdateResult {
    int virtualObservationCount = 0;
    std::vector<BBox> virtualObservations;
};

std::optional<ReupdateResult> reupdate(
    kalman::KalmanBoxTracker& filter,
    const kalman::KalmanBoxTracker& posterior,
    const BBox& lastObservation,
    int lastObservationAge,
    const BBox& currentObservation,
    int currentAge);

}  // namespace tracking::ocsort
