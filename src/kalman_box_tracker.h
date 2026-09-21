#pragma once

#include <Eigen/Dense>

#include "compat_optional.h"
#include "geometry.h"
#include "tracking/types.h"

namespace tracking {
namespace kalman {

class KalmanBoxTracker {
public:
    explicit KalmanBoxTracker(const BBox& initialBox);

    bool predict();
    bool update(const BBox& observation);
    detail::Optional<BBox> estimate() const;
    detail::Optional<geometry::Point> velocityPerFrame() const;

private:
    using StateVector = Eigen::Matrix<float, 7, 1>;
    using StateCovariance = Eigen::Matrix<float, 7, 7>;

    detail::Optional<BBox> stateToBox() const;

    StateVector state_ = StateVector::Zero();
    StateCovariance covariance_ = StateCovariance::Zero();
    bool usable_ = false;
};

}  // namespace kalman
}  // namespace tracking
