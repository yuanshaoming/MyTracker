#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "tracking/types.h"

namespace tracking {
namespace tool {

struct ReplayFrame {
    ReplayFrame() = default;
    ReplayFrame(std::int64_t frameId, std::int64_t timestamp)
        : id(frameId), timestampMs(timestamp) {}

    std::int64_t id = 0;
    std::int64_t timestampMs = 0;
};

struct ReplayInputs {
    std::vector<ReplayFrame> frames;
    std::map<std::int64_t, std::vector<Detection>> detectionsByFrame;
};

ReplayInputs parseReplayInputs(
    const std::string& framesPath,
    const std::string& detectionsPath);

}  // namespace tool
}  // namespace tracking
