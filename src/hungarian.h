#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace tracking {
namespace assignment {

struct CostMatrix {
    CostMatrix() = default;
    CostMatrix(std::size_t rowCount, std::size_t columnCount, std::vector<float> matrixValues)
        : rows(rowCount), columns(columnCount), values(std::move(matrixValues)) {}

    std::size_t rows = 0;
    std::size_t columns = 0;
    std::vector<float> values;
};

struct ValidityMask {
    ValidityMask() = default;
    ValidityMask(std::size_t rowCount, std::size_t columnCount, std::vector<bool> edgeValues)
        : rows(rowCount), columns(columnCount), values(std::move(edgeValues)) {}

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

}  // namespace assignment
}  // namespace tracking
