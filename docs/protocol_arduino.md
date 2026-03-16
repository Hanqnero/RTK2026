# UART protocol: Raspberry Pi <-> Arduino Mega

Reference firmware: [../arduino/motor_interface.ino](../arduino/motor_interface.ino).

## Link parameters

- **Baud rate**: 115200
- **Cycle**: ~100 ms (Arduino loop waits so each cycle is ~100 ms)

## Host -> Arduino (RX on Arduino): 4 bytes

| Offset | Size | Type   | Description |
|--------|------|--------|-------------|
| 0      | 1 B  | uint8  | Left motor forward PWM (0–255) |
| 1      | 1 B  | uint8  | Left motor backward PWM (0–255) |
| 2      | 1 B  | uint8  | Right motor forward PWM (0–255) |
| 3      | 1 B  | uint8  | Right motor backward PWM (0–255) |

Effective signed PWM applied to each motor: `pwm = forward - backward` (range −255..255).
To send a signed speed, set the appropriate direction byte and leave the other at 0.

## Arduino -> Host (TX from Arduino): 16 bytes

| Offset | Size | Type    | Description |
|--------|------|---------|-------------|
| 0      | 4 B  | int32_t | Left encoder speed (counts per SPEED_PERIOD_MS, from `Encoder::speed()`) |
| 4      | 4 B  | int32_t | Left encoder count (low 32 bits of `Encoder::cnt()`) |
| 8      | 4 B  | int32_t | Right encoder speed |
| 12     | 4 B  | int32_t | Right encoder count |

Byte order: little-endian. Total packet size: 16 bytes (`struct TxPacket` in firmware).

## Firmware notes

- Arduino reads exactly 4 bytes via `Serial.readBytes()` when ≥ 4 bytes are available.
- TX packet (`TxPacket`) is sent every loop cycle (~100 ms) using `Serial.write()`.
