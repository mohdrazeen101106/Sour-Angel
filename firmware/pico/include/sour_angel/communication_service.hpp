#pragma once

#include "sour_angel/setpoint_mailbox.hpp"
#include "sour_angel/telemetry_mailbox.hpp"

#include <cstdint>

namespace sour_angel::control {

// Adapter boundary for the Blitz transport implementation. Implementations
// own link setup, framing, byte order, and transport-specific validation.
class BlitzTransport {
public:
    virtual ~BlitzTransport() = default;

    // Return true only after a whole setpoint frame has been decoded and
    // transport validation has passed. Returns at most one frame per call.
    virtual bool try_receive_setpoint(SetpointCommand& command) noexcept = 0;

    // Serialize and send one telemetry frame. False indicates a transport
    // failure; it does not change the Core 0 sensor/control state.
    virtual bool send_telemetry(const TelemetryFrame& frame) noexcept = 0;
};

struct CommunicationPollResult {
    bool setpoint_received;
    bool telemetry_available;
    bool telemetry_sent;
};

struct CommunicationStats {
    std::uint32_t setpoints_received;
    std::uint32_t telemetry_sent;
    std::uint32_t telemetry_send_failures;
};

// Core 1 task scheduler: setpoint RX at 100 Hz and telemetry TX at 200 Hz.
// Call poll repeatedly from Core 1 task context with the low 32 bits of a
// monotonic microsecond clock. Missed periods are skipped, never burst-replayed.
class CommunicationService final {
public:
    static constexpr std::uint32_t kSetpointPeriodUs = 10'000U;
    static constexpr std::uint32_t kTelemetryPeriodUs = 5'000U;

    CommunicationService(SetpointMailbox& setpoints,
                         TelemetryMailbox& telemetry) noexcept;

    CommunicationPollResult poll(std::uint32_t now_us,
                                 BlitzTransport& transport) noexcept;

    CommunicationStats stats() const noexcept { return stats_; }

private:
    static bool deadline_reached(std::uint32_t now,
                                 std::uint32_t deadline) noexcept;
    static std::uint32_t next_deadline(std::uint32_t now,
                                       std::uint32_t deadline,
                                       std::uint32_t period) noexcept;

    SetpointMailbox& setpoints_;
    TelemetryMailbox& telemetry_;
    std::uint32_t next_setpoint_rx_us_{0};
    std::uint32_t next_telemetry_tx_us_{0};
    CommunicationStats stats_{};
    bool initialized_{false};
};

} // namespace sour_angel::control
