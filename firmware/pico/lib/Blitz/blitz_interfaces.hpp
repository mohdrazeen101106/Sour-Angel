enum PacketID : uint8_t {
    VELOCITY = 5,
    BNOREADING = 1,
    ENCODER = 2,
    SOUR_ANGEL_SETPOINT = 10,
    SOUR_ANGEL_TELEMETRY = 11,
};

#pragma pack(push, 1)
struct Velocity {
    float vx;
    float vy;
    float vw;
};  
#pragma pack(pop)

// Pwm struct for 4 motor PWM values
#pragma pack(push, 1)
struct Pwm {
    float m1;
    float m2;
    float m3;
    float m4;
};  
#pragma pack(pop)

#pragma pack(push, 1)
struct BnoReading {
    float current_angle;
    float refer_angle; 
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Encoder {
    int16_t enc_x; // X encoder ticks
    int16_t enc_y; // Y encoder ticks
};
#pragma pack(pop);

// Keep these field orders synchronized with ros2/sour_angel_interfaces/msg.
// estop is a byte on the wire; C++ bool is intentionally not serialized.
#pragma pack(push, 1)
struct SourAngelSetpoint {
    uint32_t sequence;
    float x_ref_m;
    float v_ref_mps;
    float phi_ref_rad;
    uint8_t estop;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SourAngelTelemetry {
    uint32_t timestamp_us;
    float theta_rad;
    float theta_dot_radps;
    float phi_rad;
    float phi_dot_radps;
    float gyro_z_radps;
    float enc_vel_l_radps;
    float enc_vel_r_radps;
    float pend_angle_rad;
    float flywheel_rpm;
    float pack_voltage_v;
    uint8_t fault_flags;
};
#pragma pack(pop)

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case VELOCITY:           return sizeof(Velocity);
        case BNOREADING:         return sizeof(BnoReading);
        case ENCODER:            return sizeof(Encoder);
        case SOUR_ANGEL_SETPOINT: return sizeof(SourAngelSetpoint);
        case SOUR_ANGEL_TELEMETRY:return sizeof(SourAngelTelemetry);
        default:                 return 0; // unknown
    }
}
