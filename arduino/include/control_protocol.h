#pragma once

#include <stdint.h>

struct __attribute__((packed)) ControlPacket {
    float target_linear_mps;
    float target_angular_rps;
};

struct __attribute__((packed)) TelemetryPacket {
    float odom_x_m;
    float odom_y_m;
    float odom_heading_rad;
};
