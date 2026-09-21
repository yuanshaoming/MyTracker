#pragma once

#include "compat_optional.h"
#include "tracking/types.h"

namespace tracking {
namespace geometry {

struct Point {
    Point() = default;
    Point(float pointX, float pointY) : x(pointX), y(pointY) {}

    float x = 0.0F;
    float y = 0.0F;
};

struct BBoxObservation {
    BBoxObservation() = default;
    BBoxObservation(float x, float y, float boxArea, float ratio)
        : centerX(x), centerY(y), area(boxArea), aspectRatio(ratio) {}

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
