#include "sour_angel/setpoint_mailbox.hpp"

namespace sour_angel::control {

SetpointMailbox::SetpointMailbox() noexcept {
    critical_section_init(&critical_section_);
}

SetpointMailbox::~SetpointMailbox() {
    critical_section_deinit(&critical_section_);
}

void SetpointMailbox::publish(const SetpointCommand& command) noexcept {
    critical_section_enter_blocking(&critical_section_);
    const std::size_t next_slot = active_slot_ ^ 1U;
    slots_[next_slot] = command;
    active_slot_ = next_slot;
    has_command_ = true;
    critical_section_exit(&critical_section_);
}

bool SetpointMailbox::snapshot(SetpointCommand& command) const noexcept {
    critical_section_enter_blocking(&critical_section_);
    const bool available = has_command_;
    if (available) {
        command = slots_[active_slot_];
    }
    critical_section_exit(&critical_section_);
    return available;
}

} // namespace sour_angel::control
