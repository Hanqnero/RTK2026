# Arduino Mega Pinout Specification

This document defines the pin mapping used by the current robot firmware on Arduino Mega 2560.

## Scope

- Board: Arduino Mega 2560 (ATmega2560)
- Firmware mapping source: include/motor_interface.h
- Core pin capability source: ArduinoCore-avr-1.8.7/variants/mega/pins_arduino.h

## Pin Assignment Table

| Subsystem | Signal | Arduino Pin | Direction | Notes |
|---|---|---:|---|---|
| Motor Driver (dual PWM) | LEFT_PWM_A | D5 | Output (PWM) | Left motor PWM input A |
| Motor Driver (dual PWM) | LEFT_PWM_B | D4 | Output (PWM) | Left motor PWM input B |
| Motor Driver (dual PWM) | RIGHT_PWM_A | D7 | Output (PWM) | Right motor PWM input A |
| Motor Driver (dual PWM) | RIGHT_PWM_B | D6 | Output (PWM) | Right motor PWM input B |
| Status LED | LED_BUILTIN | D13 | Output | Blinks once per control loop cycle |
| Left Encoder | LEFT_ENC_CLK | D16 | Input, polled | Quadrature channel A |
| Left Encoder | LEFT_ENC_DT | D18 | Input, polled | Quadrature channel B |
| Right Encoder | RIGHT_ENC_CLK | D17 | Input, polled | Quadrature channel A |
| Right Encoder | RIGHT_ENC_DT | D19 | Input, polled | Quadrature channel B |
| Sonar 0: `sonar_front_left` | TRIG / ECHO | D38 / D39 | Output / Input | First pin in pair is TRIG |
| Sonar 1: `sonar_front_right` | TRIG / ECHO | D40 / D41 | Output / Input | First pin in pair is TRIG |
| Sonar 2: `sonar_left_right` | TRIG / ECHO | D30 / D31 | Output / Input | First pin in pair is TRIG |
| Sonar 3: `sonar_left_left` | TRIG / ECHO | D32 / D33 | Output / Input | First pin in pair is TRIG |
| Sonar 4: `sonar_right_right` | TRIG / ECHO | D36 / D37 | Output / Input | First pin in pair is TRIG |
| Sonar 5: `sonar_right_left` | TRIG / ECHO | D34 / D35 | Output / Input | First pin in pair is TRIG |
| Host Serial Telemetry/Control | UART0_RX | D0 | Input | USB serial bridge to host TX |
| Host Serial Telemetry/Control | UART0_TX | D1 | Output | USB serial bridge to host RX |

## Interface Requirements

### Motor Driver

- Firmware uses GyverMotor2 `GM2::PWM_PWM_SPEED`.
- Each motor uses two PWM-capable outputs. For positive commands, one PWM input receives duty while the opposite input is held at zero; negative commands swap the active input.
- Left motor uses LEFT_PWM_A/LEFT_PWM_B.
- Right motor uses RIGHT_PWM_A/RIGHT_PWM_B.
- Previous DIR_DIR_PWM motor pins D22, D24, D26, D28, D9, and D10 are no longer used by the motor driver.

### Encoders

- All encoder inputs are configured as INPUT_PULLUP in firmware.
- The main firmware polls all four lines together; it does not attach encoder interrupts.

### Sonars

- All six HC-SR04 modules are powered from the common 5V/GND rails, never from GPIO.
- Firmware triggers only one module at a time, in the table order, to suppress cross-talk.
- Each finished measurement is sent as `SonarSamplePayload`; obstacle handling belongs to Nav2 and Collision Monitor.

### BMI270 I2C

- D20/D21 remain the Mega hardware-I2C pins. The main firmware does not read BMI270; the production IMU is connected to Raspberry Pi.
- I2C bus uses Arduino Mega hardware I2C pins:
  - SDA D20, SCL D21
- I2C clock configured in BMI270 test firmware: 400 kHz.
- BMI270 address with `SDO` tied to GND: `0x68`.

IMU board header mapping (module shown in image, silk labels left->right: `VCC`, `GND`, `SCL`, `SDA`, `SDO`, `CS`, `INT1`, `INT2`):

| IMU board pin | Firmware signal name | Arduino Mega pin | Required |
|---|---|---:|---|
| VCC | IMU power | 3.3V | Yes |
| GND | GND | GND | Yes |
| SCL | IMU_SCL | D21 / SCL | Yes |
| SDA | IMU_SDA | D20 / SDA | Yes |
| SDO | Address select | GND | Yes |
| CS | I2C mode select | 3.3V | Yes |
| INT1 | IMU_INT1 | Not used by current firmware | No |
| INT2 | IMU_INT2 | Not used by current firmware | No |

Notes:
- The tested module works as BMI270 over I2C at address `0x68`.
- `CS` must be held HIGH for I2C mode.
- `SDO` selects the I2C address; tie it to GND for `0x68`.
- If interrupts are needed later, connect `INT1` and/or `INT2` to free external-interrupt-capable Mega pins and extend firmware.

### Serial Link

- Telemetry and command packets use Serial (UART0) at 115200 baud.
- USB cable connection uses onboard USB-to-UART bridge mapped to D0/D1.

## Reserved/Used Pins Summary

- Used digital pins: D0, D1, D4, D5, D6, D7, D13, D16-D19, D30-D41
- Unused digital pins: all others (available for future expansion)
- Analog pins A0-A15: currently unused

## Power and Ground Guidance

- Use a common ground between Arduino, motor driver, encoders, and IMU.
- Power motors from their external supply through the two BTS7960 modules.
- Tie Arduino, both BTS7960 modules and the sonar supply to a common ground.
- Do not power motors directly from Arduino 5V rail.

## Change Control

If any mapping changes in include/motor_interface.h, update this document and reflash firmware to keep wiring and software aligned.
