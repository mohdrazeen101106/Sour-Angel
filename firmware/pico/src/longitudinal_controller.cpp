#include "sour_angel/longitudinal_controller.hpp"

#include <algorithm>
#include <cmath>

namespace sour_angel::control {
namespace {

bool finite(const LongitudinalState& state) noexcept {
    return std::isfinite(state.position_m) &&
           std::isfinite(state.velocity_mps) &&
           std::isfinite(state.pitch_rad) &&
           std::isfinite(state.pitch_rate_radps);
}

bool finite(const SafeSetpoint& setpoint) noexcept {
    return std::isfinite(setpoint.x_ref_m) &&
           std::isfinite(setpoint.v_ref_mps) &&
           std::isfinite(setpoint.phi_ref_rad);
}

} // namespace

LongitudinalControlResult LongitudinalController::compute(
    const LongitudinalState& state,
    const SafeSetpoint& setpoint) const noexcept {
    if (!finite(state) || !finite(setpoint)) {
        return LongitudinalControlResult{0.0F, false, false};
    }

    if (!setpoint.active) {
        return LongitudinalControlResult{0.0F, true, false};
    }

    const float position_error_m = state.position_m - setpoint.x_ref_m;
    const float velocity_error_mps = state.velocity_mps - setpoint.v_ref_mps;
    const float unconstrained_torque_nm = -(
        kPositionGain * position_error_m +
        kVelocityGain * velocity_error_mps +
        kPitchGain * state.pitch_rad +
        kPitchRateGain * state.pitch_rate_radps);
    const float torque_nm = std::clamp(
        unconstrained_torque_nm, -kMaxDriveTorqueNm, kMaxDriveTorqueNm);

    if (!std::isfinite(torque_nm)) {
        return LongitudinalControlResult{0.0F, false, false};
    }

    return LongitudinalControlResult{torque_nm, true, true};
}

} // namespace sour_angel::control
