#pragma once

#include <memory>

#include "tracking/itracker.h"
#include "tracking/tracker_config.h"

namespace tracking {

class SortTracker final : public ITracker {
public:
    explicit SortTracker(TrackerConfig config = {});
    ~SortTracker() override;

    std::vector<TrackResult> update(
        const std::vector<Detection>& detections,
        std::int64_t timestampMs) override;
    void reset() override;

private:
    struct Impl;

    std::unique_ptr<Impl> impl_;
};

}  // namespace tracking
