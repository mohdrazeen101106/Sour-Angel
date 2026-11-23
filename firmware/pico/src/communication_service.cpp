#include "sour_angel/communication_service.hpp"

namespace sour_angel::control {

CommunicationService::CommunicationService(SetpointMailbox& setpoints,
                                           TelemetryMailbox& telemetry) noexcept
    : setpoints_(setpoints), telemetry_(telemetry) {}

bool CommunicationService::deadline_reached(const std::uint32_t now,
                                            const std::uint32_t deadline) noexcept {
    // Valid when the service is polled at least once within half the uint32
    // timer range (about 35 minutes), which is comfortably longer than normal
    // firmware scheduling intervals.
    return (now - deadline) < 0x80000000U;
}

std::uint32_t CommunicationService::next_deadline(const std::uint32_t now,
                                                  const std::uint32_t deadline,
                                                  const std::uint32_t period) noexcept {
    const std::uint32_t lateness = now - deadline;
    const std::uint32_t elapsed_periods = (lateness / period) + 1U;
    return deadline + (elapsed_periods * period);
}

CommunicationPollResult CommunicationService::poll(const std::uint32_t now_us,
                                                     BlitzTransport& transport) noexcept {
    if (!initialized_) {
        next_setpoint_rx_us_ = now_us;
        next_telemetry_tx_us_ = now_us;
        initialized_ = true;
    }

    CommunicationPollResult result{false, false, false};

    if (deadline_reached(now_us, next_setpoint_rx_us_)) {
        SetpointCommand command{};
        if (transport.try_receive_setpoint(command)) {
            setpoints_.publish(command);
            result.setpoint_received = true;
            ++stats_.setpoints_received;
        }
        next_setpoint_rx_us_ = next_deadline(
            now_us, next_setpoint_rx_us_, kSetpointPeriodUs);
    }

    if (deadline_reached(now_us, next_telemetry_tx_us_)) {
        TelemetryFrame frame{};
        if (telemetry_.snapshot(frame)) {
            result.telemetry_available = true;
            result.telemetry_sent = transport.send_telemetry(frame);
            if (result.telemetry_sent) {
                ++stats_.telemetry_sent;
            } else {
                ++stats_.telemetry_send_failures;
            }
        }
        next_telemetry_tx_us_ = next_deadline(
            now_us, next_telemetry_tx_us_, kTelemetryPeriodUs);
    }

    return result;
}

} // namespace sour_angel::control
