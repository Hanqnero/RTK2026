#pragma once

#include <stdint.h>

struct __attribute__((packed)) ControlPacket {
    float target_linear_mps;
    float target_angular_rps;
    uint8_t debug_raw_encoder;
};

struct __attribute__((packed)) TelemetryPacket {
    float odom_x_m;
    float odom_y_m;
    float odom_heading_rad;
    int32_t raw_left_encoder_delta;
    int32_t raw_right_encoder_delta;
    int16_t left_pwm;
    int16_t right_pwm;
};
