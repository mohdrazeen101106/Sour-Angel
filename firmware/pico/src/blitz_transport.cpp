#include "sour_angel/blitz_transport.hpp"

#include <blitz.hpp>

#include <cmath>
#include <cstddef>
#include <type_traits>

namespace sour_angel::control {
namespace {

static_assert(sizeof(float) == 4, "Blitz packets require 32-bit float");
static_assert(std::is_trivially_copyable<SourAngelSetpoint>::value,
              "Blitz setpoint payload must be byte-copyable");
static_assert(std::is_trivially_copyable<SourAngelTelemetry>::value,
              "Blitz telemetry payload must be byte-copyable");
static_assert(sizeof(SourAngelSetpoint) == 17,
              "Keep Blitz setpoint payload aligned with ROS fields");
static_assert(sizeof(SourAngelTelemetry) == 45,
              "Keep Blitz telemetry payload aligned with ROS fields");

bool finite_payload(const SourAngelSetpoint& packet) noexcept {
    return std::isfinite(packet.x_ref_m) &&
           std::isfinite(packet.v_ref_mps) &&
           std::isfinite(packet.phi_ref_rad) &&
           packet.estop <= 1U;
}

} // namespace

void BlitzSerialTransport::begin(const unsigned long baud_rate) noexcept {
    Serial.begin(baud_rate);
}

bool BlitzSerialTransport::try_receive_setpoint(SetpointCommand& command) noexcept {
    // Blitz returns a vector containing the packet ID followed by the complete
    // configured payload. Check both before calling its struct parser.
    auto packet = receive_data();
    if (packet.size() != sizeof(SourAngelSetpoint) + 1U ||
        packet.front() != SOUR_ANGEL_SETPOINT) {
        return false;
    }

    const SourAngelSetpoint decoded = parse_struct<SourAngelSetpoint>(packet);
    if (!finite_payload(decoded)) {
        return false;
    }

    command = SetpointCommand{
        decoded.sequence,
        decoded.x_ref_m,
        decoded.v_ref_mps,
        decoded.phi_ref_rad,
        decoded.estop != 0U,
    };
    return true;
}

bool BlitzSerialTransport::send_telemetry(const TelemetryFrame& frame) noexcept {
    if (!Serial) {
        return false;
    }

    // Leave enough room for the sync byte plus the entire library packet so
    // send_data() never starts a partial frame because its TX queue is full.
    constexpr std::size_t kWireBytes = sizeof(SourAngelTelemetry) + 2U;
    const int tx_capacity = Serial.availableForWrite();
    if (tx_capacity < 0 || static_cast<std::size_t>(tx_capacity) < kWireBytes) {
        return false;
    }

    const SourAngelTelemetry packet{
        frame.timestamp_us,
        frame.theta_rad,
        frame.theta_dot_radps,
        frame.phi_rad,
        frame.phi_dot_radps,
        frame.gyro_z_radps,
        frame.enc_vel_l_radps,
        frame.enc_vel_r_radps,
        frame.pend_angle_rad,
        frame.flywheel_rpm,
        frame.pack_voltage_v,
        frame.fault_flags,
    };

    send_data(pack_data(packet, SOUR_ANGEL_TELEMETRY));
    return true;
}

} // namespace sour_angel::control
