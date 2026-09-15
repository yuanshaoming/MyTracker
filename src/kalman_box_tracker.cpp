#include "kalman_box_tracker.h"

#include <stdexcept>

#include <Eigen/Cholesky>

#include "geometry.h"

namespace tracking::kalman {
namespace {

using MeasurementMatrix = Eigen::Matrix<float, 4, 7>;
using MeasurementVector = Eigen::Matrix<float, 4, 1>;
using MeasurementCovariance = Eigen::Matrix<float, 4, 4>;
using KalmanGain = Eigen::Matrix<float, 7, 4>;
using StateCovariance = Eigen::Matrix<float, 7, 7>;
using StateVector = Eigen::Matrix<float, 7, 1>;

StateVector observationVector(const geometry::BBoxObservation& observation) {
    StateVector state = StateVector::Zero();
    state << observation.centerX, observation.centerY, observation.area, observation.aspectRatio,
        0.0F, 0.0F, 0.0F;
    return state;
}

MeasurementVector measurementVector(const geometry::BBoxObservation& observation) {
    MeasurementVector measurement;
    measurement << observation.centerX, observation.centerY, observation.area, observation.aspectRatio;
    return measurement;
}

StateCovariance transitionMatrix() {
    StateCovariance transition = StateCovariance::Identity();
    transition(0, 4) = 1.0F;
    transition(1, 5) = 1.0F;
    transition(2, 6) = 1.0F;
    return transition;
}

MeasurementMatrix measurementMatrix() {
    MeasurementMatrix measurement = MeasurementMatrix::Zero();
    measurement(0, 0) = 1.0F;
    measurement(1, 1) = 1.0F;
    measurement(2, 2) = 1.0F;
    measurement(3, 3) = 1.0F;
    return measurement;
}

StateCovariance processNoise() {
    StateCovariance noise = StateCovariance::Zero();
    noise.diagonal() << 1.0F, 1.0F, 1.0F, 1.0F, 0.01F, 0.01F, 0.01F;
    return noise;
}

MeasurementCovariance measurementNoise() {
    return MeasurementCovariance::Identity();
}

StateCovariance initialCovariance() {
    StateCovariance covariance = StateCovariance::Zero();
    covariance.diagonal() << 10.0F, 10.0F, 10.0F, 10.0F, 100.0F, 100.0F, 100.0F;
    return covariance;
}

}  // namespace

KalmanBoxTracker::KalmanBoxTracker(const BBox& initialBox) {
    const auto observation = geometry::toObservation(initialBox);
    if (!observation) {
        throw std::invalid_argument("initial box must be finite with positive area and aspect ratio");
    }

    state_ = observationVector(*observation);
    covariance_ = initialCovariance();
    usable_ = true;
}

bool KalmanBoxTracker::predict() {
    if (!usable_) {
        return false;
    }

    const auto transition = transitionMatrix();
    state_ = transition * state_;
    covariance_ = transition * covariance_ * transition.transpose() + processNoise();
    if (!state_.allFinite() || !stateToBox()) {
        usable_ = false;
        return false;
    }
    return true;
}

bool KalmanBoxTracker::update(const BBox& observationBox) {
    if (!usable_) {
        return false;
    }

    const auto observation = geometry::toObservation(observationBox);
    if (!observation) {
        return false;
    }

    const auto measurement = measurementMatrix();
    const MeasurementVector innovation = measurementVector(*observation) - measurement * state_;
    const MeasurementCovariance innovationCovariance =
        measurement * covariance_ * measurement.transpose() + measurementNoise();
    Eigen::LDLT<MeasurementCovariance> decomposition(innovationCovariance);
    if (decomposition.info() != Eigen::Success) {
        return false;
    }

    const KalmanGain gain = decomposition.solve(
        (covariance_ * measurement.transpose()).transpose()).transpose();
    if (!gain.allFinite()) {
        return false;
    }

    state_ += gain * innovation;
    const StateCovariance identity = StateCovariance::Identity();
    const StateCovariance residual = identity - gain * measurement;
    covariance_ = residual * covariance_ * residual.transpose() +
                  gain * measurementNoise() * gain.transpose();
    if (!state_.allFinite() || !covariance_.allFinite() || !stateToBox()) {
        usable_ = false;
        return false;
    }
    return true;
}

std::optional<BBox> KalmanBoxTracker::estimate() const {
    if (!usable_) {
        return std::nullopt;
    }
    return stateToBox();
}

std::optional<geometry::Point> KalmanBoxTracker::velocityPerFrame() const {
    if (!usable_ || !state_.allFinite()) {
        return std::nullopt;
    }
    return geometry::Point{state_(4), state_(5)};
}

std::optional<BBox> KalmanBoxTracker::stateToBox() const {
    if (!state_.allFinite()) {
        return std::nullopt;
    }

    return geometry::fromObservation(geometry::BBoxObservation{
        state_(0),
        state_(1),
        state_(2),
        state_(3),
    });
}

}  // namespace tracking::kalman
