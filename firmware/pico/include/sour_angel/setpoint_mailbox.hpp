#pragma once

#include "pico/critical_section.h"
#include "sour_angel/setpoint_safety.hpp"

#include <array>
#include <cstddef>

namespace sour_angel::control {

// Double-buffered Core 1 -> Core 0 handoff for complete, decoded commands.
// The short Pico SDK critical section makes slot selection and copying atomic
// across the two cores. Construct after Pico SDK runtime initialization.
class SetpointMailbox final {
public:
    SetpointMailbox() noexcept;
    ~SetpointMailbox();

    SetpointMailbox(const SetpointMailbox&) = delete;
    SetpointMailbox& operator=(const SetpointMailbox&) = delete;
    SetpointMailbox(SetpointMailbox&&) = delete;
    SetpointMailbox& operator=(SetpointMailbox&&) = delete;

    // Call from the Core 1 communication task after transport validation.
    void publish(const SetpointCommand& command) noexcept;

    // Call from the Core 0 control task. Returns the latest complete command,
    // or false before Core 1 has published one.
    bool snapshot(SetpointCommand& command) const noexcept;

private:
    mutable critical_section_t critical_section_{};
    std::array<SetpointCommand, 2> slots_{};
    std::size_t active_slot_{0};
    bool has_command_{false};
};

} // namespace sour_angel::control
