# Flat sensor state from the Pico. timestamp_us is the Pico's wrapping clock.
uint32 timestamp_us
float32 theta_rad
float32 theta_dot_radps
float32 phi_rad
float32 phi_dot_radps
float32 gyro_z_radps
float32 enc_vel_l_radps
float32 enc_vel_r_radps
float32 pend_angle_rad
float32 flywheel_rpm
float32 pack_voltage_v
uint8 fault_flags
