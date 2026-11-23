# Pico communications and setpoint safety

This firmware segment establishes the application boundary between the Jetson
setpoints, Pico sensor telemetry, and the real-time controller. The target
firmware uses the Arduino-Pico core through PlatformIO. `BlitzSerialTransport`
adapts the checked-in Blitz library to the communication scheduler.

## Data flow

- Arduino-Pico runs `setup()` / `loop()` on Core 0 for sensor acquisition and
  stabilization, and `setup1()` / `loop1()` on Core 1 for communications and
  watchdog supervision.
- Core 1 polls `BlitzSerialTransport::try_receive_setpoint` at 100 Hz. The adapter
  returns only complete commands that Blitz has decoded and the adapter has validated
  and validated. The service publishes each command to `SetpointMailbox`.
- Core 0 snapshots the latest command, passes it to `SetpointSafety::receive`,
  and calls `SetpointSafety::current` each control cycle before computing
  motor commands.
- Core 0 publishes a complete `TelemetryFrame` after updating sensors. Core 1
  snapshots and sends the latest frame at 200 Hz.

The telemetry fields and fault-bit meanings match the architecture document.
`TelemetryFrame` is an in-memory type, not a wire ABI: the adapter maps fields
to the packed Blitz payload. A transport failure does not mutate sensor or
control state. Setpoint sequence validation and the 200 ms freshness timeout
remain in `SetpointSafety`; ignored frames do not refresh the timeout.
The sensor producer is not implemented yet, so the telemetry mailbox remains
empty until a later sensor bring-up segment publishes its first frame.

## Concurrency and safety

Both mailboxes use Pico SDK `pico_sync` critical sections around only the
double-buffer copy and active-slot switch. Do not perform serial I/O, logging,
or control calculations while holding a mailbox lock. Construct the mailboxes
after the Pico SDK runtime starts, and call the communication service only
from the Core 1 task.

No accepted command, software estop, external safety stop, or 200 ms without a
new valid sequence produces zero references. Roll commands are limited to
±15 degrees. The hardware-specific controller still owns torque shutdown and
safe motor outputs.

## Blitz transport

The private library in `lib/Blitz` provides framing and serialization. Sour
Angel adds its packet IDs and packed payload types in
`lib/Blitz/blitz_interfaces.hpp`; the adapter calls Blitz's `receive_data`,
`parse_struct`, `pack_data`, and `send_data` functions. It checks packet ID,
decoded length, finite command values, and the estop byte before publishing a
setpoint. Core 0 still owns sequence ordering, command freshness, and the roll
limit.

Blitz accesses Arduino `Serial`, so this implementation uses the Pico's USB
CDC connection to the Jetson, not a GPIO UART. The baud argument is retained
for Arduino API compatibility; USB CDC does not use it as a physical line
rate.

## Build

Install PlatformIO Core and build the Arduino-Pico firmware from this
directory:

```sh
pio run -e sour-angel-pico
```

The target selects the Arduino-Pico core because it provides `setup1()` /
`loop1()` for the RP2040 second core while retaining access to Pico SDK
facilities such as PIO. PlatformIO configuration is in `platformio.ini`.

The standalone CMake target remains available for host compilation of the
hardware-independent safety library. The optional mailbox/service sources use
Pico SDK `pico_sync` and are compiled as part of the PlatformIO firmware.

The ROS flat message schema is in `ros2/sour_angel_interfaces`. Sensor drivers,
telemetry production, wireless kill input, and the stabilization controller
remain later implementation segments.

The hardware-independent longitudinal LQR calculation is implemented in
`LongitudinalController`. It uses the architecture's documented gains and
13.9 N·m torque ceiling, returns zero for inactive/stale safe setpoints, and
does not drive pins. It requires position, velocity, chassis pitch, and pitch
rate inputs from the future sensor/state-estimation layer.

`LateralController` implements the documented lateral gains, 0.15 Hz torque
split, 2/6 degree override bands, pendulum stop guard, and momentum-wheel
speed guard. It reports when the wheel reaches the 1200 rpm desaturation
threshold; the architecture does not specify a desaturation gain, so this
slice reports the condition without inventing a bias value. Its torque outputs
still require measured roll/pendulum state and a later motor-driver layer.
Gear-backlash compensation and motor current conversion belong in that
actuator layer.

`ControlStateEstimator` now converts fresh sensor snapshots into both LQR
inputs. It applies the shell-speed correction
`R * ((enc_vel_l + enc_vel_r) / 2 + theta_dot)` and trapezoidally integrates
along-track position. It rejects non-finite samples, re-anchors after gaps
over 20 ms without integrating missing time, and marks state older than 10 ms
stale. Core 0 consumes valid estimates and evaluates both torque-request
modules; the motor output layer is still pending.
The ROS 2 flat messages are in `ros2/sour_angel_interfaces`; the bridge maps
those fields into the standard `sensor_msgs/Imu`, `nav_msgs/Odometry`, and
diagnostics topics without nested custom message paths.
