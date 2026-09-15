#pragma once

#include <optional>

#include <Eigen/Dense>

#include "geometry.h"
#include "tracking/types.h"

namespace tracking::kalman {

class KalmanBoxTracker {
public:
    explicit KalmanBoxTracker(const BBox& initialBox);

    bool predict();
    bool update(const BBox& observation);
    std::optional<BBox> estimate() const;
    std::optional<geometry::Point> velocityPerFrame() const;

private:
    using StateVector = Eigen::Matrix<float, 7, 1>;
    using StateCovariance = Eigen::Matrix<float, 7, 7>;

    std::optional<BBox> stateToBox() const;

    StateVector state_ = StateVector::Zero();
    StateCovariance covariance_ = StateCovariance::Zero();
    bool usable_ = false;
};

}  // namespace tracking::kalman
