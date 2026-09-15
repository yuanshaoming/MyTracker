#include "geometry.h"

#include <algorithm>
#include <cmath>

namespace tracking::geometry {
namespace {

std::optional<float> finiteFloat(double value) noexcept {
    if (!std::isfinite(value)) {
        return std::nullopt;
    }

    const float result = static_cast<float>(value);
    if (!std::isfinite(result)) {
        return std::nullopt;
    }
    return result;
}

std::optional<Point> makePoint(double x, double y) noexcept {
    const auto pointX = finiteFloat(x);
    const auto pointY = finiteFloat(y);
    if (!pointX || !pointY) {
        return std::nullopt;
    }
    return Point{*pointX, *pointY};
}

}  // namespace

bool isValid(const BBox& bbox) noexcept {
    return std::isfinite(bbox.x1) && std::isfinite(bbox.y1) &&
           std::isfinite(bbox.x2) && std::isfinite(bbox.y2) &&
           bbox.x2 > bbox.x1 && bbox.y2 > bbox.y1;
}

float area(const BBox& bbox) noexcept {
    if (!isValid(bbox)) {
        return 0.0F;
    }

    const double width = static_cast<double>(bbox.x2) - bbox.x1;
    const double height = static_cast<double>(bbox.y2) - bbox.y1;
    return finiteFloat(width * height).value_or(0.0F);
}

std::optional<Point> center(const BBox& bbox) noexcept {
    if (!isValid(bbox)) {
        return std::nullopt;
    }

    return makePoint(
        (static_cast<double>(bbox.x1) + bbox.x2) * 0.5,
        (static_cast<double>(bbox.y1) + bbox.y2) * 0.5);
}

float iou(const BBox& first, const BBox& second) noexcept {
    if (!isValid(first) || !isValid(second)) {
        return 0.0F;
    }

    const double intersectionLeft = std::max(static_cast<double>(first.x1), static_cast<double>(second.x1));
    const double intersectionTop = std::max(static_cast<double>(first.y1), static_cast<double>(second.y1));
    const double intersectionRight = std::min(static_cast<double>(first.x2), static_cast<double>(second.x2));
    const double intersectionBottom = std::min(static_cast<double>(first.y2), static_cast<double>(second.y2));

    if (intersectionRight <= intersectionLeft || intersectionBottom <= intersectionTop) {
        return 0.0F;
    }

    const double intersectionArea =
        (intersectionRight - intersectionLeft) * (intersectionBottom - intersectionTop);
    const double firstArea =
        (static_cast<double>(first.x2) - first.x1) * (static_cast<double>(first.y2) - first.y1);
    const double secondArea =
        (static_cast<double>(second.x2) - second.x1) * (static_cast<double>(second.y2) - second.y1);
    const double unionArea = firstArea + secondArea - intersectionArea;

    if (!std::isfinite(unionArea) || unionArea <= 0.0) {
        return 0.0F;
    }

    return finiteFloat(intersectionArea / unionArea).value_or(0.0F);
}

std::optional<float> centerDistance(const BBox& first, const BBox& second) noexcept {
    const auto firstCenter = center(first);
    const auto secondCenter = center(second);
    if (!firstCenter || !secondCenter) {
        return std::nullopt;
    }

    const double deltaX = static_cast<double>(secondCenter->x) - firstCenter->x;
    const double deltaY = static_cast<double>(secondCenter->y) - firstCenter->y;
    return finiteFloat(std::hypot(deltaX, deltaY));
}

std::optional<Point> unitDirection(const BBox& from, const BBox& to) noexcept {
    const auto fromCenter = center(from);
    const auto toCenter = center(to);
    if (!fromCenter || !toCenter) {
        return std::nullopt;
    }

    const double deltaX = static_cast<double>(toCenter->x) - fromCenter->x;
    const double deltaY = static_cast<double>(toCenter->y) - fromCenter->y;
    const double length = std::hypot(deltaX, deltaY);
    if (!std::isfinite(length)) {
        return std::nullopt;
    }
    if (length == 0.0) {
        return Point{};
    }

    return makePoint(deltaX / length, deltaY / length);
}

std::optional<BBoxObservation> toObservation(const BBox& bbox) noexcept {
    if (!isValid(bbox)) {
        return std::nullopt;
    }

    const double width = static_cast<double>(bbox.x2) - bbox.x1;
    const double height = static_cast<double>(bbox.y2) - bbox.y1;
    const auto centerPoint = center(bbox);
    const auto boxArea = finiteFloat(width * height);
    const auto aspectRatio = finiteFloat(width / height);
    if (!centerPoint || !boxArea || !aspectRatio || *boxArea <= 0.0F || *aspectRatio <= 0.0F) {
        return std::nullopt;
    }

    return BBoxObservation{centerPoint->x, centerPoint->y, *boxArea, *aspectRatio};
}

std::optional<BBox> fromObservation(const BBoxObservation& observation) noexcept {
    if (!std::isfinite(observation.centerX) || !std::isfinite(observation.centerY) ||
        !std::isfinite(observation.area) || !std::isfinite(observation.aspectRatio) ||
        observation.area <= 0.0F || observation.aspectRatio <= 0.0F) {
        return std::nullopt;
    }

    const double width = std::sqrt(
        static_cast<double>(observation.area) * observation.aspectRatio);
    const double height = std::sqrt(
        static_cast<double>(observation.area) / observation.aspectRatio);
    const auto x1 = finiteFloat(static_cast<double>(observation.centerX) - width * 0.5);
    const auto y1 = finiteFloat(static_cast<double>(observation.centerY) - height * 0.5);
    const auto x2 = finiteFloat(static_cast<double>(observation.centerX) + width * 0.5);
    const auto y2 = finiteFloat(static_cast<double>(observation.centerY) + height * 0.5);
    if (!x1 || !y1 || !x2 || !y2) {
        return std::nullopt;
    }

    const BBox bbox{*x1, *y1, *x2, *y2};
    if (!isValid(bbox)) {
        return std::nullopt;
    }
    return bbox;
}

}  // namespace tracking::geometry
