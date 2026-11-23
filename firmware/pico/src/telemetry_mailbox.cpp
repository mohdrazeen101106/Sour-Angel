#include "sour_angel/telemetry_mailbox.hpp"

namespace sour_angel::control {

TelemetryMailbox::TelemetryMailbox() noexcept {
    critical_section_init(&critical_section_);
}

TelemetryMailbox::~TelemetryMailbox() {
    critical_section_deinit(&critical_section_);
}

void TelemetryMailbox::publish(const TelemetryFrame& frame) noexcept {
    critical_section_enter_blocking(&critical_section_);
    const std::size_t next_slot = active_slot_ ^ 1U;
    slots_[next_slot] = frame;
    active_slot_ = next_slot;
    has_frame_ = true;
    critical_section_exit(&critical_section_);
}

bool TelemetryMailbox::snapshot(TelemetryFrame& frame) const noexcept {
    critical_section_enter_blocking(&critical_section_);
    const bool available = has_frame_;
    if (available) {
        frame = slots_[active_slot_];
    }
    critical_section_exit(&critical_section_);
    return available;
}

} // namespace sour_angel::control
