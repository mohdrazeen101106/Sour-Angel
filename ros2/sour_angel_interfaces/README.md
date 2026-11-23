# Sour Angel flat ROS 2 interfaces

These messages are the flat field schema used by the Blitz ROS bridge. The
bridge maps each field explicitly to the fixed-size packet payload; message
memory layout is not the UART wire layout.

| ROS message | Direction | Purpose |
| --- | --- | --- |
| `PicoSetpoint` | Jetson → Pico | Sequence, longitudinal references, roll reference, software estop |
| `PicoTelemetry` | Pico → Jetson | Pico timestamp, IMU/encoder/pendulum/flywheel/battery state, fault bits |
| `PicoStatus` | Pico → Jetson | Flat command freshness, kill, battery, stall, and transport status |

All fields are scalar and top-level. This keeps Blitz field-path configuration
compatible with the library's flat-field ROS mapping. The Blitz bridge
publishes raw `PicoTelemetry`; a Sour Angel state bridge maps it to standard
IMU, odometry, and diagnostics topics for consumers such as
`robot_localization`.

Build this package inside a ROS 2 workspace with `colcon build --packages-up-to
sour_angel_interfaces`.

The Blitz ROS package maps these top-level fields to its configured fixed
packet fields in `ros2/blitz/blitz/interfaces.py`. Its message IDs, field
order, format string, and MCU packet sizes must be kept in sync with
`firmware/pico/lib/Blitz/blitz_interfaces.hpp`. The transport implementation
remains the Blitz library.

The checked-in Pico definitions currently use:

| Packet ID | Direction | ROS message | Blitz `struct_fmt` | Payload bytes |
| --- | --- | --- | --- | ---: |
| `10` | Jetson → Pico | `PicoSetpoint` | `IfffB` | 17 |
| `11` | Pico → Jetson | `PicoTelemetry` | `I10fB` | 45 |

The setpoint fields are `sequence, x_ref_m, v_ref_mps, phi_ref_rad, estop`.
Telemetry is `timestamp_us`, the ten float fields in `.msg` order, then
`fault_flags`. Blitz adds its sync marker and packet ID outside these payload
formats.

`PicoStatus` is a bridge-side status view, not a third MCU packet in this
segment. The MCU wire payloads are packed field sequences; they are not ROS
message memory images.
