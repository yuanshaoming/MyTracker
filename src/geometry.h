#pragma once

#include "compat_optional.h"
#include "tracking/types.h"

namespace tracking {
namespace geometry {

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
detail::Optional<Point> center(const BBox& bbox) noexcept;
float iou(const BBox& first, const BBox& second) noexcept;
detail::Optional<float> centerDistance(const BBox& first, const BBox& second) noexcept;
detail::Optional<Point> unitDirection(const BBox& from, const BBox& to) noexcept;
detail::Optional<BBoxObservation> toObservation(const BBox& bbox) noexcept;
detail::Optional<BBox> fromObservation(const BBoxObservation& observation) noexcept;

}  // namespace geometry
}  // namespace tracking
