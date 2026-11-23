from sour_angel_interfaces.msg import PicoSetpoint, PicoTelemetry
from blitz import Blitz


# Flat Sour Angel interfaces. IDs, formats, and field order match
# firmware/pico/lib/Blitz/blitz_interfaces.hpp.
blitz_interfaces = {
    "sour_angel_setpoint": Blitz(
        topic="/control/cmd_setpoints",
        msg_id=10,
        struct_fmt="IfffB",
        fields=[
            "sequence",
            "x_ref_m",
            "v_ref_mps",
            "phi_ref_rad",
            "estop",
        ],
        ros_msg=PicoSetpoint,
        from_mcu=False,
    ),
    "sour_angel_telemetry": Blitz(
        topic="/pico/telemetry",
        msg_id=11,
        struct_fmt="I10fB",
        fields=[
            "timestamp_us",
            "theta_rad",
            "theta_dot_radps",
            "phi_rad",
            "phi_dot_radps",
            "gyro_z_radps",
            "enc_vel_l_radps",
            "enc_vel_r_radps",
            "pend_angle_rad",
            "flywheel_rpm",
            "pack_voltage_v",
            "fault_flags",
        ],
        ros_msg=PicoTelemetry,
        from_mcu=True,
    ),
}
