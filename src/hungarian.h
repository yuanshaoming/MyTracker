#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace tracking::assignment {

struct CostMatrix {
    std::size_t rows = 0;
    std::size_t columns = 0;
    std::vector<float> values;
};

struct ValidityMask {
    std::size_t rows = 0;
    std::size_t columns = 0;
    std::vector<bool> values;
};

struct AssignmentResult {
    std::vector<std::pair<std::size_t, std::size_t>> matches;
    std::vector<std::size_t> unmatchedTracks;
    std::vector<std::size_t> unmatchedDetections;
};

AssignmentResult solve(
    const CostMatrix& costs,
    const ValidityMask& validEdges);

}  // namespace tracking::assignment
