# Sour Angel Blitz ROS bridge

This package uses the checked-in Blitz ROS bridge and its normal serial
packer/parser nodes. Sour Angel's field mapping is configured in
`blitz/interfaces.py`; the payload formats and packet IDs match the Pico's
`firmware/pico/lib/Blitz/blitz_interfaces.hpp` definitions.

Build the interfaces before launching the bridge:

```sh
colcon build --packages-up-to blitz
source install/setup.bash
ros2 launch blitz blitz.launch.py
```

The host sends `PicoSetpoint` on `/control/cmd_setpoints` and receives raw
`PicoTelemetry` on `/pico/telemetry`. A later Sour Angel state bridge maps the
raw telemetry into standard IMU, odometry, and diagnostics topics. The
`PicoStatus` message is derived on the host and is not sent as a separate Blitz
packet.
