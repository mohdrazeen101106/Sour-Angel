#include "sour_angel/lateral_controller.hpp"

#include <cmath>

namespace sour_angel::control {
namespace {

bool finite(const LateralState& state) noexcept {
    return std::isfinite(state.roll_rad) &&
           std::isfinite(state.roll_rate_radps) &&
           std::isfinite(state.pendulum_angle_rad) &&
           std::isfinite(state.flywheel_rpm);
}

bool finite(const SafeSetpoint& setpoint) noexcept {
    return std::isfinite(setpoint.x_ref_m) &&
           std::isfinite(setpoint.v_ref_mps) &&
           std::isfinite(setpoint.phi_ref_rad);
}

LateralControlResult stopped_result(const bool input_valid,
                                    const bool enabled) noexcept {
    return LateralControlResult{
        0.0F, 0.0F, 0.0F, input_valid, enabled, false, false, false};
}

bool pendulum_pushes_outward(const float angle_rad,
                             const float torque_nm) noexcept {
    return (angle_rad >= LateralController::kPendulumStopRad && torque_nm > 0.0F) ||
           (angle_rad <= -LateralController::kPendulumStopRad && torque_nm < 0.0F);
}

bool wheel_pushes_faster(const float rpm, const float torque_nm) noexcept {
    return std::fabs(rpm) >= LateralController::kWheelMaximumRpm &&
           rpm * torque_nm > 0.0F;
}

} // namespace

LateralControlResult LateralController::compute(
    const LateralState& state,
    const SafeSetpoint& setpoint,
    const float dt_seconds) noexcept {
    if (!finite(state) || !finite(setpoint) ||
        !std::isfinite(dt_seconds) || dt_seconds <= 0.0F ||
        dt_seconds > kMaximumStepSeconds) {
        reset();
        return stopped_result(false, false);
    }

    if (!setpoint.active) {
        reset();
        return stopped_result(true, false);
    }

    const float roll_error_rad = state.roll_rad - setpoint.phi_ref_rad;
    const float total_torque_nm = -(
        kRollGain * roll_error_rad + kRollRateGain * state.roll_rate_radps);
    if (!std::isfinite(total_torque_nm)) {
        reset();
        return stopped_result(false, false);
    }

    float pendulum_torque_nm = 0.0F;
    float wheel_torque_nm = total_torque_nm;
    const float absolute_error_rad = std::fabs(roll_error_rad);
    bool pendulum_at_stop = false;
    bool wheel_at_speed = false;

    if (absolute_error_rad < kWheelOnlyErrorRad) {
        // Near balance, reserve the pendulum for slow steering and use the
        // momentum wheel for roll damping.
        slow_torque_nm_ = 0.0F;
    } else if (absolute_error_rad > kPendulumOnlyErrorRad) {
        // Large roll errors call for the pendulum at full rate.
        pendulum_torque_nm = total_torque_nm;
        wheel_torque_nm = 0.0F;
        slow_torque_nm_ = total_torque_nm;
    } else {
        const float alpha = 1.0F - std::exp(
            -6.2831853072F * kCrossoverHz * dt_seconds);
        slow_torque_nm_ += alpha * (total_torque_nm - slow_torque_nm_);
        pendulum_torque_nm = slow_torque_nm_;
        wheel_torque_nm = total_torque_nm - pendulum_torque_nm;
    }

    if (!std::isfinite(pendulum_torque_nm) || !std::isfinite(wheel_torque_nm)) {
        reset();
        return stopped_result(false, false);
    }

    if (pendulum_pushes_outward(state.pendulum_angle_rad, pendulum_torque_nm)) {
        pendulum_torque_nm = 0.0F;
        wheel_torque_nm = total_torque_nm;
        pendulum_at_stop = true;
    }

    if (wheel_pushes_faster(state.flywheel_rpm, wheel_torque_nm)) {
        wheel_torque_nm = 0.0F;
        if (!pendulum_pushes_outward(state.pendulum_angle_rad, total_torque_nm)) {
            pendulum_torque_nm = total_torque_nm;
        } else {
            pendulum_torque_nm = 0.0F;
            pendulum_at_stop = true;
        }
        wheel_at_speed = true;
    }

    return LateralControlResult{
        total_torque_nm,
        pendulum_torque_nm,
        wheel_torque_nm,
        true,
        true,
        std::fabs(state.flywheel_rpm) >= kWheelDesaturationRpm,
        pendulum_at_stop,
        wheel_at_speed,
    };
}

void LateralController::reset() noexcept {
    slow_torque_nm_ = 0.0F;
}

} // namespace sour_angel::control
