#pragma once

#include <stdint.h>

struct __attribute__((packed)) ControlPacket {
    float target_linear_mps;
    float target_angular_rps;
};

struct __attribute__((packed)) TelemetryPacket {
    float target_linear_mps;
    float target_angular_rps;
    float current_linear_mps;
    float current_angular_rps;
    float target_left_wheel_mps;
    float target_right_wheel_mps;
    float current_left_wheel_mps;
    float current_right_wheel_mps;
    int16_t left_pwm;
    int16_t right_pwm;
    int32_t left_count;
    int32_t right_count;
};
