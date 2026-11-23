#pragma once

#include "sour_angel/setpoint_safety.hpp"

#include <cstdint>

namespace sour_angel::control {

struct LateralState {
    float roll_rad;
    float roll_rate_radps;
    float pendulum_angle_rad;
    float flywheel_rpm;
};

struct LateralControlResult {
    float total_roll_torque_nm;
    float pendulum_torque_nm;
    float wheel_torque_nm;
    bool input_valid;
    bool enabled;
    bool wheel_desaturation_needed;
    bool pendulum_blocked_at_stop;
    bool wheel_blocked_at_speed;
};

// Stateful 0.15 Hz allocation of lateral roll torque. It produces torque
// requests only; motor current limits and ESC commands belong to hardware I/O.
class LateralController final {
public:
    static constexpr float kRollGain = 113.5F;
    static constexpr float kRollRateGain = 44.0F;
    static constexpr float kCrossoverHz = 0.15F;
    static constexpr float kWheelOnlyErrorRad = 0.0349065850F; // 2 degrees
    static constexpr float kPendulumOnlyErrorRad = 0.1047197551F; // 6 degrees
    static constexpr float kPendulumStopRad = 0.7853981634F; // 45 degrees
    static constexpr float kWheelDesaturationRpm = 1200.0F;
    static constexpr float kWheelMaximumRpm = 2000.0F;
    static constexpr float kMaximumStepSeconds = 0.05F;

    LateralControlResult compute(const LateralState& state,
                                 const SafeSetpoint& setpoint,
                                 float dt_seconds) noexcept;

    void reset() noexcept;

private:
    float slow_torque_nm_{0.0F};
};

} // namespace sour_angel::control
