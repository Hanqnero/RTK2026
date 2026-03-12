# UART protocol: Raspberry Pi <-> Arduino Mega

Reference firmware: [../arduino/motor_interface.ino](../arduino/motor_interface.ino).

## Link parameters

- **Baud rate**: 115200
- **Cycle**: ~100 ms (Arduino loop waits so each cycle is ~100 ms)

## Host -> Arduino (RX on Arduino): 2 bytes

| Offset | Size | Description |
|--------|------|-------------|
| 0 | 1 B | Left motor speed (PWM). Interpretation: 0 = stop; sign/direction and scale as in firmware (e.g. 0..255 forward, or signed -128..127). |
| 1 | 1 B | Right motor speed (PWM). Same interpretation as left. |

Note: Firmware uses `right_set_speed(int pwm)` with the second byte; both bytes are passed as (int8_t) for signed PWM.

## Arduino -> Host (TX from Arduino): 32 bytes

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| 0 | 4 B | int32_t | Left encoder speed (counts per SPEED_PERIOD_MS, from Encoder::speed()) |
| 4 | 4 B | int64_t low 32b or int32_t | Left encoder count (from Encoder::cnt()) |
| 8 | 4 B | int32_t | Right encoder speed |
| 12 | 4 B | int64_t low 32b or int32_t | Right encoder count |
| 16 | 16 B | - | Reserved (zero or unused) |

Encoder period is 100 ms (SPEED_PERIOD_MS in encoder.cpp). Byte order: assume little-endian for multi-byte fields.

## Firmware notes

- TX buffer is filled in `loop()` with left_speed, left_cnt (low 32b), right_speed, right_cnt (little-endian int32) from the encoder objects.
