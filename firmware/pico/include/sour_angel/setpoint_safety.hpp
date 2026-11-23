#pragma once

#include <cstdint>

namespace sour_angel::control {

// Application-level view of the Jetson setpoint frame. Packet framing and
// transport validation belong to the Blitz adapter, not this module.
struct SetpointCommand {
    std::uint32_t sequence;
    float x_ref_m;
    float v_ref_mps;
    float phi_ref_rad;
    bool estop;
};

// Safe command supplied to the real-time controller. A stopped command has
// zero references and active == false; motor output policy remains in the
// hardware-specific control loop.
struct SafeSetpoint {
    float x_ref_m;
    float v_ref_mps;
    float phi_ref_rad;
    bool active;
};

class SetpointSafety final {
public:
    static constexpr std::uint32_t kCommandTimeoutMs = 200;
    static constexpr float kMaxRollRad = 0.2617993878F; // 15 degrees

    // Call for each completely decoded and transport-validated frame. The timestamp
    // is a monotonic millisecond clock; unsigned subtraction permits rollover.
    // Returns false for duplicate/old sequence numbers or non-finite values.
    bool receive(const SetpointCommand& command, std::uint32_t now_ms) noexcept;

    // Call from the control loop. A stale link, estop, or external safety stop
    // yields zero references immediately.
    SafeSetpoint current(std::uint32_t now_ms,
                         bool external_safety_stop = false) const noexcept;

private:
    static bool sequence_is_newer(std::uint32_t candidate,
                                  std::uint32_t previous) noexcept;
    static bool finite(const SetpointCommand& command) noexcept;

    SetpointCommand command_{};
    std::uint32_t received_at_ms_{0};
    bool has_command_{false};
};

} // namespace sour_angel::control
