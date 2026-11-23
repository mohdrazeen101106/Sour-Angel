#include "sour_angel/control_state_estimator.hpp"

#include <cmath>

namespace sour_angel::control {
namespace {

bool finite_sensor_state(const TelemetryFrame& sample) noexcept {
    return std::isfinite(sample.theta_rad) &&
           std::isfinite(sample.theta_dot_radps) &&
           std::isfinite(sample.phi_rad) &&
           std::isfinite(sample.phi_dot_radps) &&
           std::isfinite(sample.enc_vel_l_radps) &&
           std::isfinite(sample.enc_vel_r_radps) &&
           std::isfinite(sample.pend_angle_rad) &&
           std::isfinite(sample.flywheel_rpm);
}

} // namespace

void ControlStateEstimator::reset(const float initial_position_m) noexcept {
    position_m_ = std::isfinite(initial_position_m) ? initial_position_m : 0.0F;
    longitudinal_ = LongitudinalState{};
    lateral_ = LateralState{};
    last_sample_us_ = 0U;
    initialized_ = false;
    state_valid_ = false;
}

bool ControlStateEstimator::update(const TelemetryFrame& sample) noexcept {
    if (!finite_sensor_state(sample)) {
        state_valid_ = false;
        return false;
    }

    const float mean_encoder_rate_radps =
        0.5F * (sample.enc_vel_l_radps + sample.enc_vel_r_radps);
    const float shell_rate_radps = mean_encoder_rate_radps + sample.theta_dot_radps;
    const float shell_speed_mps = kShellRadiusM * shell_rate_radps;
    if (!std::isfinite(shell_speed_mps)) {
        state_valid_ = false;
        return false;
    }

    if (!initialized_) {
        last_sample_us_ = sample.timestamp_us;
        longitudinal_ = LongitudinalState{
            position_m_, shell_speed_mps, sample.theta_rad, sample.theta_dot_radps};
        lateral_ = LateralState{
            sample.phi_rad, sample.phi_dot_radps,
            sample.pend_angle_rad, sample.flywheel_rpm};
        initialized_ = true;
        state_valid_ = true;
        return true;
    }

    const std::uint32_t elapsed_us = sample.timestamp_us - last_sample_us_;
    if (elapsed_us == 0U) {
        return false;
    }

    last_sample_us_ = sample.timestamp_us;

    if (elapsed_us > kMaximumSampleGapUs) {
        // Re-anchor time and velocity without integrating across missing data.
        state_valid_ = false;
        longitudinal_.velocity_mps = shell_speed_mps;
        longitudinal_.pitch_rad = sample.theta_rad;
        longitudinal_.pitch_rate_radps = sample.theta_dot_radps;
        lateral_ = LateralState{
            sample.phi_rad, sample.phi_dot_radps,
            sample.pend_angle_rad, sample.flywheel_rpm};
        return false;
    }

    const float dt_seconds = static_cast<float>(elapsed_us) * 1.0e-6F;
    position_m_ += 0.5F * (longitudinal_.velocity_mps + shell_speed_mps) * dt_seconds;
    if (!std::isfinite(position_m_)) {
        state_valid_ = false;
        return false;
    }

    longitudinal_ = LongitudinalState{
        position_m_, shell_speed_mps, sample.theta_rad, sample.theta_dot_radps};
    lateral_ = LateralState{
        sample.phi_rad, sample.phi_dot_radps,
        sample.pend_angle_rad, sample.flywheel_rpm};
    state_valid_ = true;
    return true;
}

bool ControlStateEstimator::snapshot(const std::uint32_t now_us,
                                     ControlStateEstimate& estimate) const noexcept {
    if (!initialized_ || !state_valid_ ||
        (now_us - last_sample_us_) > kMaximumStateAgeUs) {
        return false;
    }

    estimate = ControlStateEstimate{longitudinal_, lateral_};
    return true;
}

} // namespace sour_angel::control
