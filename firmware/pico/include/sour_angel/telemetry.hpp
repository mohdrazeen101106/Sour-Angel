#pragma once

#include <cstdint>

namespace sour_angel::control {

// In-memory representation of the architecture's Pico -> Jetson telemetry.
// The Blitz adapter must serialize these fields explicitly; this struct is not
// a wire-format or ABI contract.
struct TelemetryFrame {
    std::uint32_t timestamp_us;
    float theta_rad;
    float theta_dot_radps;
    float phi_rad;
    float phi_dot_radps;
    float gyro_z_radps;
    float enc_vel_l_radps;
    float enc_vel_r_radps;
    float pend_angle_rad;
    float flywheel_rpm;
    float pack_voltage_v;
    std::uint8_t fault_flags;
};

namespace telemetry_fault {
inline constexpr std::uint8_t kHardwareKill = 1U << 0U;
inline constexpr std::uint8_t kLowVoltageWarning = 1U << 1U;
inline constexpr std::uint8_t kLowVoltageCritical = 1U << 2U;
inline constexpr std::uint8_t kMotorStall = 1U << 3U;
} // namespace telemetry_fault

} // namespace sour_angel::control
