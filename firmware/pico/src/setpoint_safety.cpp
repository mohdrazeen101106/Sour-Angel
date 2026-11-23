#include "sour_angel/setpoint_safety.hpp"

#include <cmath>

namespace sour_angel::control {

bool SetpointSafety::sequence_is_newer(const std::uint32_t candidate,
                                       const std::uint32_t previous) noexcept {
    // RFC-1982-style serial arithmetic: the half-range difference is
    // ambiguous, so only forward differences below 2^31 are accepted.
    const std::uint32_t difference = candidate - previous;
    return difference != 0U && difference < 0x80000000U;
}

bool SetpointSafety::finite(const SetpointCommand& command) noexcept {
    return std::isfinite(command.x_ref_m) &&
           std::isfinite(command.v_ref_mps) &&
           std::isfinite(command.phi_ref_rad);
}

bool SetpointSafety::receive(const SetpointCommand& command,
                             const std::uint32_t now_ms) noexcept {
    if (!finite(command)) {
        return false;
    }

    if (has_command_ && !sequence_is_newer(command.sequence, command_.sequence)) {
        return false;
    }

    command_ = command;
    received_at_ms_ = now_ms;
    has_command_ = true;
    return true;
}

SafeSetpoint SetpointSafety::current(const std::uint32_t now_ms,
                                    const bool external_safety_stop) const noexcept {
    const bool timed_out = !has_command_ ||
        (now_ms - received_at_ms_) >= kCommandTimeoutMs;

    if (external_safety_stop || timed_out || command_.estop) {
        return SafeSetpoint{0.0F, 0.0F, 0.0F, false};
    }

    const float limited_phi = std::fmax(
        -kMaxRollRad,
        std::fmin(command_.phi_ref_rad, kMaxRollRad));

    return SafeSetpoint{
        command_.x_ref_m,
        command_.v_ref_mps,
        limited_phi,
        true,
    };
}

} // namespace sour_angel::control
