# UART protocol: Raspberry Pi <-> Arduino Mega

Reference firmware: [../arduino/motor_interface/motor_interface.ino](../arduino/motor_interface/motor_interface.ino).
Bridge node: [../src/rtk2026_driver/rtk2026_driver/arduino_bridge_node.py](../src/rtk2026_driver/rtk2026_driver/arduino_bridge_node.py).

## Link parameters

- **Baud rate**: 115200
- **Cycle**: ~100 ms (Arduino loop waits so each cycle is ~100 ms)

## Host -> Arduino (RX on Arduino): 4 bytes

| Offset | Size | Description |
|--------|------|-------------|
| 0 | 1 B | Left motor forward PWM (0..255) |
| 1 | 1 B | Left motor backward PWM (0..255) |
| 2 | 1 B | Right motor forward PWM (0..255) |
| 3 | 1 B | Right motor backward PWM (0..255) |

Arduino computes: `left_pwm = rx[0] - rx[1]`, `right_pwm = rx[2] - rx[3]`, result in range -255..255.
Bridge splits signed PWM: positive -> forward byte, negative -> backward byte.

## Arduino -> Host (TX from Arduino): 16 bytes

Packed struct `TxPacket`:

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| 0 | 4 B | int32_t | Left encoder speed (counts per cycle) |
| 4 | 4 B | int32_t | Left encoder count (cumulative) |
| 8 | 4 B | int32_t | Right encoder speed (counts per cycle) |
| 12 | 4 B | int32_t | Right encoder count (cumulative) |

Byte order: little-endian. Bridge reads with `struct.unpack("<iiii", raw)`.

## Notes

- Encoders are optional: if `enc_start()` is not called, speed and count fields will be zero.
- Motor PWM range on Arduino: `constrain(pwm, -255, 255)`.
- Bridge PWM range: -255..255 (clamped before splitting into forward/backward bytes).
