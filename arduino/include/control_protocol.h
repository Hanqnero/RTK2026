#pragma once

#include <stdint.h>

struct __attribute__((packed)) ControlPacket {
    float target_linear_mps;
    float target_angular_rps;
};

struct __attribute__((packed)) TelemetryPacket {
    uint8_t imu_online;
    uint8_t imu_chip_id;
    int16_t imu_acc_x;
    int16_t imu_acc_y;
    int16_t imu_acc_z;
    int16_t imu_gyro_x;
    int16_t imu_gyro_y;
    int16_t imu_gyro_z;
    float odom_x_m;
    float odom_y_m;
    float odom_heading_rad;
};
