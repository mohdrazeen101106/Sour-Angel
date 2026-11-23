#pragma once

#include "sour_angel/communication_service.hpp"

namespace sour_angel::control {

// Adapter over the checked-in Blitz MCU library. Blitz owns framing,
// serialization, and access to Arduino Serial (USB CDC on the Pico).
class BlitzSerialTransport final : public BlitzTransport {
public:
    void begin(unsigned long baud_rate) noexcept;

    bool try_receive_setpoint(SetpointCommand& command) noexcept override;
    bool send_telemetry(const TelemetryFrame& frame) noexcept override;
};

} // namespace sour_angel::control
