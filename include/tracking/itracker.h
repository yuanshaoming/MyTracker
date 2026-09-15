#pragma once

#include <cstdint>
#include <vector>

#include "tracking/types.h"

namespace tracking {

class ITracker {
public:
    virtual ~ITracker() = default;

    virtual std::vector<TrackResult> update(
        const std::vector<Detection>& detections,
        std::int64_t timestampMs) = 0;
    virtual void reset() = 0;
};

}  // namespace tracking
