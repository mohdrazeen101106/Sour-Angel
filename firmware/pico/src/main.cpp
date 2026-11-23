#include <Arduino.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>

#include "sour_angel/blitz_transport.hpp"
#include "sour_angel/control_state_estimator.hpp"
#include "sour_angel/lateral_controller.hpp"
#include "sour_angel/longitudinal_controller.hpp"
#include "sour_angel/setpoint_safety.hpp"
#include "sour_angel/setpoint_mailbox.hpp"
#include "sour_angel/telemetry_mailbox.hpp"

// Core allocation follows the architecture reference:
//   Core 0 (setup/loop): sensor acquisition and 400 Hz stabilization.
//   Core 1 (setup1/loop1): Blitz USB CDC, 200 Hz telemetry, 100 Hz command RX,
//                         wireless kill input, and watchdog supervision.
// This slice wires communications and setpoint freshness; motor control,
// sensors, and the physical kill input remain hardware bring-up work.
namespace {

using sour_angel::control::CommunicationService;
using sour_angel::control::ControlStateEstimate;
using sour_angel::control::ControlStateEstimator;
using sour_angel::control::LateralController;
using sour_angel::control::LongitudinalController;
using sour_angel::control::SetpointCommand;
using sour_angel::control::SetpointMailbox;
using sour_angel::control::SetpointSafety;
using sour_angel::control::TelemetryMailbox;

constexpr std::uint32_t kControlPeriodUs = 2'500U;
alignas(SetpointMailbox) std::byte setpoint_mailbox_storage[sizeof(SetpointMailbox)];
alignas(TelemetryMailbox) std::byte telemetry_mailbox_storage[sizeof(TelemetryMailbox)];
alignas(CommunicationService) std::byte communication_service_storage[sizeof(CommunicationService)];
alignas(sour_angel::control::BlitzSerialTransport)
    std::byte blitz_transport_storage[sizeof(sour_angel::control::BlitzSerialTransport)];
SetpointMailbox* setpoint_mailbox = nullptr;
TelemetryMailbox* telemetry_mailbox = nullptr;
CommunicationService* communication_service = nullptr;
sour_angel::control::BlitzSerialTransport* blitz_transport = nullptr;
std::atomic<bool> mailboxes_ready{false};
SetpointSafety setpoint_safety;
ControlStateEstimator control_state_estimator;
LongitudinalController longitudinal_controller;
LateralController lateral_controller;

} // namespace

void setup() {
    // Mailboxes initialize Pico SDK multicore locks; construct after the
    // Arduino-Pico runtime has initialized.
    setpoint_mailbox = new (setpoint_mailbox_storage) SetpointMailbox();
    telemetry_mailbox = new (telemetry_mailbox_storage) TelemetryMailbox();
    control_state_estimator.reset();
    mailboxes_ready.store(true, std::memory_order_release);
}

void loop() {
    const std::uint32_t cycle_start_us = micros();
    static std::uint32_t previous_cycle_us = 0U;
    static bool has_previous_cycle = false;
    const std::uint32_t elapsed_cycle_us = has_previous_cycle
        ? cycle_start_us - previous_cycle_us
        : kControlPeriodUs;
    previous_cycle_us = cycle_start_us;
    has_previous_cycle = true;

    sour_angel::control::TelemetryFrame sensor_sample{};
    if (telemetry_mailbox->snapshot(sensor_sample)) {
        (void)control_state_estimator.update(sensor_sample);
    }

    SetpointCommand command{};
    if (setpoint_mailbox->snapshot(command)) {
        (void)setpoint_safety.receive(command, millis());
    }

    // The forthcoming controller consumes this safe command before producing
    // motor outputs. For now, evaluate the 200 ms command watchdog every cycle.
    const auto safe_command = setpoint_safety.current(millis());
    ControlStateEstimate control_state{};
    if (control_state_estimator.snapshot(micros(), control_state)) {
        const auto longitudinal_demand = longitudinal_controller.compute(
            control_state.longitudinal, safe_command);
        const float dt_seconds = static_cast<float>(elapsed_cycle_us) * 1.0e-6F;
        const auto lateral_demand = lateral_controller.compute(
            control_state.lateral, safe_command, dt_seconds);
        // Motor and ESC output drivers consume these requests in the next
        // hardware integration slice.
        (void)longitudinal_demand;
        (void)lateral_demand;
    } else {
        lateral_controller.reset();
    }

    const std::uint32_t elapsed_us = micros() - cycle_start_us;
    if (elapsed_us < kControlPeriodUs) {
        delayMicroseconds(kControlPeriodUs - elapsed_us);
    }
}

void setup1() {
    while (!mailboxes_ready.load(std::memory_order_acquire)) {
        delayMicroseconds(50);
    }

    blitz_transport = new (blitz_transport_storage)
        sour_angel::control::BlitzSerialTransport();
    blitz_transport->begin(115200U);
    communication_service = new (communication_service_storage)
        CommunicationService(*setpoint_mailbox, *telemetry_mailbox);
}

void loop1() {
    if (communication_service == nullptr || blitz_transport == nullptr) {
        delayMicroseconds(50);
        return;
    }
    (void)communication_service->poll(micros(), *blitz_transport);
}
