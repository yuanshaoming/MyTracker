#include "hungarian.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "compat_optional.h"

namespace tracking {
namespace assignment {
namespace {

struct LexCost {
    std::int64_t unmatchedCount = 0;
    long double cost = 0.0L;
};

bool lessThan(const LexCost& left, const LexCost& right) noexcept {
    return left.unmatchedCount < right.unmatchedCount ||
           (left.unmatchedCount == right.unmatchedCount && left.cost < right.cost);
}

LexCost add(const LexCost& left, const LexCost& right) noexcept {
    return LexCost{
        left.unmatchedCount + right.unmatchedCount,
        left.cost + right.cost,
    };
}

LexCost subtract(const LexCost& left, const LexCost& right) noexcept {
    return LexCost{
        left.unmatchedCount - right.unmatchedCount,
        left.cost - right.cost,
    };
}

std::size_t checkedProduct(std::size_t rows, std::size_t columns) {
    if (columns != 0 && rows > std::numeric_limits<std::size_t>::max() / columns) {
        throw std::invalid_argument("matrix dimensions overflow");
    }
    return rows * columns;
}

void validateInput(const CostMatrix& costs, const ValidityMask& validEdges) {
    const std::size_t elementCount = checkedProduct(costs.rows, costs.columns);
    if (costs.values.size() != elementCount) {
        throw std::invalid_argument("cost matrix size does not match its dimensions");
    }
    if (validEdges.rows != costs.rows || validEdges.columns != costs.columns) {
        throw std::invalid_argument("validity mask dimensions do not match the cost matrix");
    }
    if (validEdges.values.size() != elementCount) {
        throw std::invalid_argument("validity mask size does not match its dimensions");
    }

    for (std::size_t index = 0; index < elementCount; ++index) {
        if (validEdges.values[index] && !std::isfinite(costs.values[index])) {
            throw std::invalid_argument("a valid edge must have a finite cost");
        }
    }
}

}  // namespace

AssignmentResult solve(const CostMatrix& costs, const ValidityMask& validEdges) {
    validateInput(costs, validEdges);

    const std::size_t trackCount = costs.rows;
    const std::size_t detectionCount = costs.columns;
    if (trackCount > std::numeric_limits<std::size_t>::max() - detectionCount) {
        throw std::invalid_argument("assignment dimensions overflow");
    }
    const std::size_t size = trackCount + detectionCount;

    AssignmentResult result;
    if (size == 0) {
        return result;
    }

    const std::size_t assignmentElements = checkedProduct(size, size);
    std::vector<detail::Optional<LexCost>> assignmentCosts(assignmentElements);
    const auto assignmentIndex = [size](std::size_t row, std::size_t column) {
        return row * size + column;
    };

    for (std::size_t track = 0; track < trackCount; ++track) {
        for (std::size_t detection = 0; detection < detectionCount; ++detection) {
            const std::size_t inputIndex = track * detectionCount + detection;
            if (validEdges.values[inputIndex]) {
                assignmentCosts[assignmentIndex(track, detection)] = LexCost{
                    0,
                    static_cast<long double>(costs.values[inputIndex]),
                };
            }
        }
        for (std::size_t dummyColumn = detectionCount; dummyColumn < size; ++dummyColumn) {
            assignmentCosts[assignmentIndex(track, dummyColumn)] = LexCost{1, 0.0L};
        }
    }

    for (std::size_t dummyRow = trackCount; dummyRow < size; ++dummyRow) {
        for (std::size_t detection = 0; detection < detectionCount; ++detection) {
            assignmentCosts[assignmentIndex(dummyRow, detection)] = LexCost{1, 0.0L};
        }
        for (std::size_t dummyColumn = detectionCount; dummyColumn < size; ++dummyColumn) {
            assignmentCosts[assignmentIndex(dummyRow, dummyColumn)] = LexCost{};
        }
    }

    std::vector<LexCost> rowPotential(size + 1);
    std::vector<LexCost> columnPotential(size + 1);
    std::vector<std::size_t> matchedRowForColumn(size + 1);
    std::vector<std::size_t> predecessorColumn(size + 1);

    for (std::size_t row = 1; row <= size; ++row) {
        matchedRowForColumn[0] = row;
        std::size_t column = 0;
        std::vector<detail::Optional<LexCost>> minimumValues(size + 1);
        std::vector<bool> used(size + 1, false);

        do {
            used[column] = true;
            const std::size_t currentRow = matchedRowForColumn[column];
            detail::Optional<LexCost> delta;
            std::size_t nextColumn = 0;

            for (std::size_t candidate = 1; candidate <= size; ++candidate) {
                if (used[candidate]) {
                    continue;
                }

                const auto& edge = assignmentCosts[assignmentIndex(currentRow - 1, candidate - 1)];
                if (!edge) {
                    continue;
                }

                const LexCost reducedCost = subtract(
                    subtract(*edge, rowPotential[currentRow]),
                    columnPotential[candidate]);
                if (!minimumValues[candidate] || lessThan(reducedCost, *minimumValues[candidate])) {
                    minimumValues[candidate] = reducedCost;
                    predecessorColumn[candidate] = column;
                }
            }

            for (std::size_t candidate = 1; candidate <= size; ++candidate) {
                if (used[candidate] || !minimumValues[candidate]) {
                    continue;
                }
                if (!delta || lessThan(*minimumValues[candidate], *delta)) {
                    delta = minimumValues[candidate];
                    nextColumn = candidate;
                }
            }

            if (!delta) {
                throw std::logic_error("assignment has no feasible augmentation");
            }
            for (std::size_t candidate = 0; candidate <= size; ++candidate) {
                if (used[candidate]) {
                    rowPotential[matchedRowForColumn[candidate]] =
                        add(rowPotential[matchedRowForColumn[candidate]], *delta);
                    columnPotential[candidate] = subtract(columnPotential[candidate], *delta);
                } else if (minimumValues[candidate]) {
                    minimumValues[candidate] = subtract(*minimumValues[candidate], *delta);
                }
            }
            column = nextColumn;
        } while (matchedRowForColumn[column] != 0);

        do {
            const std::size_t previousColumn = predecessorColumn[column];
            matchedRowForColumn[column] = matchedRowForColumn[previousColumn];
            column = previousColumn;
        } while (column != 0);
    }

    std::vector<std::size_t> columnForRow(size + 1);
    for (std::size_t column = 1; column <= size; ++column) {
        columnForRow[matchedRowForColumn[column]] = column;
    }

    std::vector<bool> matchedDetections(detectionCount, false);
    for (std::size_t track = 0; track < trackCount; ++track) {
        const std::size_t column = columnForRow[track + 1];
        if (column <= detectionCount) {
            result.matches.emplace_back(track, column - 1);
            matchedDetections[column - 1] = true;
        } else {
            result.unmatchedTracks.push_back(track);
        }
    }
    for (std::size_t detection = 0; detection < detectionCount; ++detection) {
        if (!matchedDetections[detection]) {
            result.unmatchedDetections.push_back(detection);
        }
    }

    return result;
}

}  // namespace assignment
}  // namespace tracking
