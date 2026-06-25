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

Total size: 32 bytes

Calculation:
- `float` x5 = 20 bytes
- `int32_t` x2 = 8 bytes
- `int16_t` x2 = 4 bytes
- Total = 32 bytes

## Binary Layout

| Offset | Size | Type    | Field            | Unit / Meaning |
|-------:|-----:|---------|------------------|----------------|
| 0      | 4    | float   | odom_x_m         | Odometry X position in meters |
| 4      | 4    | float   | odom_y_m         | Odometry Y position in meters |
| 8      | 4    | float   | odom_heading_rad | Odometry heading in radians, normalized to [-pi, pi] |
| 12     | 4    | int32_t | raw_left_encoder_delta | Raw left encoder delta for the last control cycle |
| 16     | 4    | int32_t | raw_right_encoder_delta | Raw right encoder delta for the last control cycle |
| 20     | 2    | int16_t | left_pwm | Left motor PWM command for the last control cycle |
| 22     | 2    | int16_t | right_pwm | Right motor PWM command for the last control cycle |
| 24     | 4    | float   | current_linear_mps | Current body linear speed in meters per second |
| 28     | 4    | float   | current_angular_rps | Current body angular speed in radians per second |

## Semantics

- Odometry is computed onboard from wheel encoder data only.
- When `ControlPacket.debug_raw_encoder` is nonzero, raw encoder deltas are copied into telemetry.
- PWM commands are reported as the final clamped values written to the motor driver.
- Current speeds are computed onboard from encoder deltas using the same no-gearbox-ratio wheel model as odometry.
- Heading uses radians and is wrapped to the interval [-pi, pi].

## Host Parsing Guidance

Because the stream is unframed fixed-size binary:

1. Read exactly 32 bytes per packet.
2. Interpret fields using little-endian types at the offsets above.
3. If byte alignment is lost, resynchronize by scanning for plausible records, for example:
  - `odom_x_m` and `odom_y_m` should be finite values within expected workspace bounds
  - `odom_heading_rad` typically in [-3.2, 3.2]
  - `current_linear_mps` and `current_angular_rps` should be finite and within the expected command range
  - raw encoder deltas are usually small integers relative to the control period

## Versioning Recommendation

This packet currently has no explicit version byte or header. If future compatibility is required, add:
- a fixed preamble (for example 2 bytes),
- a version field,
- and optionally a CRC.
