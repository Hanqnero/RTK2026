# Telemetry Packet Specification

This document defines the binary telemetry packet emitted by the Arduino firmware.

## Scope

- Source definition: `TelemetryPacket` in `include/control_protocol.h`
- Producer: `Serial.write(reinterpret_cast<const uint8_t*>(&telemetry_packet), sizeof(TelemetryPacket));` in `src/main.cpp`
- Emission period: every control cycle (`kControlPeriodMs = 100`), nominally 10 Hz

## Transport

- Physical link: UART serial
- Baud rate: 115200 (`kSerialBaudRate`)
- Packet framing: none (raw fixed-size binary records)
- Endianness: little-endian (AVR ATmega2560)
- Float format: IEEE-754 32-bit (`float`)
- Struct packing: packed (`__attribute__((packed))`)

## Packet Size

Total size: 26 bytes

Calculation:
- `uint8_t` x2 = 2 bytes
- `int16_t` x6 = 12 bytes
- `float` x3 = 12 bytes
- Total = 26 bytes

## Binary Layout

| Offset | Size | Type    | Field            | Unit / Meaning |
|-------:|-----:|---------|------------------|----------------|
| 0      | 1    | uint8_t | imu_online       | 1 if IMU is online, else 0 |
| 1      | 1    | uint8_t | imu_chip_id      | BMI270 chip ID (expected 0x24 when online) |
| 2      | 2    | int16_t | imu_acc_x        | Raw accelerometer X counts |
| 4      | 2    | int16_t | imu_acc_y        | Raw accelerometer Y counts |
| 6      | 2    | int16_t | imu_acc_z        | Raw accelerometer Z counts |
| 8      | 2    | int16_t | imu_gyro_x       | Raw gyroscope X counts |
| 10     | 2    | int16_t | imu_gyro_y       | Raw gyroscope Y counts |
| 12     | 2    | int16_t | imu_gyro_z       | Raw gyroscope Z counts |
| 14     | 4    | float   | odom_x_m         | Odometry X position in meters |
| 18     | 4    | float   | odom_y_m         | Odometry Y position in meters |
| 22     | 4    | float   | odom_heading_rad | Odometry heading in radians, normalized to [-pi, pi] |

## Semantics

- `imu_online` is derived from the IMU driver's online status.
- When IMU sampling fails in a cycle, IMU axis fields are set to 0 by firmware.
- Odometry is computed onboard from wheel encoder linear velocity and IMU-assisted yaw fusion.
- Heading uses radians and is wrapped to the interval [-pi, pi].

## Conversion Notes

Current firmware constants for interpretation:
- Gyro sensitivity assumption: `kImuGyroDpsPerLsb = 500.0 / 32768.0`
- Convert gyro counts to rad/s:
  - `gyro_rad_s = raw_gyro * kImuGyroDpsPerLsb * (pi / 180)`

Accelerometer conversion scale depends on configured BMI270 range and is currently transmitted as raw counts.

## Host Parsing Guidance

Because the stream is unframed fixed-size binary:

1. Read exactly 26 bytes per packet.
2. Interpret fields using little-endian types at the offsets above.
3. If byte alignment is lost, resynchronize by scanning for plausible records, for example:
   - `imu_online` in {0,1}
   - `imu_chip_id` often 0x24 when online
   - `odom_heading_rad` typically in [-3.2, 3.2]

## Versioning Recommendation

This packet currently has no explicit version byte or header. If future compatibility is required, add:
- a fixed preamble (for example 2 bytes),
- a version field,
- and optionally a CRC.
