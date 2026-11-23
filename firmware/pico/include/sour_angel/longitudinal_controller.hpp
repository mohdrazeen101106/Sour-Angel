#pragma once

#include "sour_angel/setpoint_safety.hpp"

namespace sour_angel::control {

struct LongitudinalState {
    float position_m;
    float velocity_mps;
    float pitch_rad;
    float pitch_rate_radps;
};

struct LongitudinalControlResult {
    float drive_torque_nm;
    bool input_valid;
    bool enabled;
};

// Architecture-defined longitudinal LQR. This module returns a bounded
// torque request; a later motor-driver layer converts it to current/PWM.
class LongitudinalController final {
public:
    static constexpr float kPositionGain = 28.0F;
    static constexpr float kVelocityGain = 97.1F;
    static constexpr float kPitchGain = 107.7F;
    static constexpr float kPitchRateGain = 22.4F;
    static constexpr float kMaxDriveTorqueNm = 13.9F;

    LongitudinalControlResult compute(const LongitudinalState& state,
                                     const SafeSetpoint& setpoint) const noexcept;
};

} // namespace sour_angel::control
