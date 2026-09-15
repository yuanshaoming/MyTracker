#pragma once

#include <optional>

#include "tracking/types.h"

namespace tracking::geometry {

struct Point {
    float x = 0.0F;
    float y = 0.0F;
};

struct BBoxObservation {
    float centerX = 0.0F;
    float centerY = 0.0F;
    float area = 0.0F;
    float aspectRatio = 0.0F;
};

bool isValid(const BBox& bbox) noexcept;
float area(const BBox& bbox) noexcept;
std::optional<Point> center(const BBox& bbox) noexcept;
float iou(const BBox& first, const BBox& second) noexcept;
std::optional<float> centerDistance(const BBox& first, const BBox& second) noexcept;
std::optional<Point> unitDirection(const BBox& from, const BBox& to) noexcept;
std::optional<BBoxObservation> toObservation(const BBox& bbox) noexcept;
std::optional<BBox> fromObservation(const BBoxObservation& observation) noexcept;

}  // namespace tracking::geometry
