#pragma once

#include "pico/critical_section.h"
#include "sour_angel/telemetry.hpp"

#include <array>
#include <cstddef>

namespace sour_angel::control {

// Double-buffered Core 0 -> Core 1 handoff for the latest sensor snapshot.
class TelemetryMailbox final {
public:
    TelemetryMailbox() noexcept;
    ~TelemetryMailbox();

    TelemetryMailbox(const TelemetryMailbox&) = delete;
    TelemetryMailbox& operator=(const TelemetryMailbox&) = delete;
    TelemetryMailbox(TelemetryMailbox&&) = delete;
    TelemetryMailbox& operator=(TelemetryMailbox&&) = delete;

    // Publish after a complete Core 0 sensor update.
    void publish(const TelemetryFrame& frame) noexcept;

    // Copy the most recently published frame. Returns false before first use.
    bool snapshot(TelemetryFrame& frame) const noexcept;

private:
    mutable critical_section_t critical_section_{};
    std::array<TelemetryFrame, 2> slots_{};
    std::size_t active_slot_{0};
    bool has_frame_{false};
};

} // namespace sour_angel::control
