# Arduino Mega Pinout Specification

This document defines the pin mapping used by the current robot firmware on Arduino Mega 2560.

## Scope

- Board: Arduino Mega 2560 (ATmega2560)
- Firmware mapping source: include/motor_interface.h
- Core pin capability source: ArduinoCore-avr-1.8.7/variants/mega/pins_arduino.h

## Pin Assignment Table

| Subsystem | Signal | Arduino Pin | Direction | Notes |
|---|---|---:|---|---|
| Motor Driver (dual PWM) | LEFT_PWM_A | D7 | Output (PWM) | Left motor PWM input A |
| Motor Driver (dual PWM) | LEFT_PWM_B | D6 | Output (PWM) | Left motor PWM input B |
| Motor Driver (dual PWM) | RIGHT_PWM_A | D5 | Output (PWM) | Right motor PWM input A |
| Motor Driver (dual PWM) | RIGHT_PWM_B | D4 | Output (PWM) | Right motor PWM input B |
| Status LED | LED_BUILTIN | D13 | Output | Blinks once per control loop cycle |
| Left Encoder | LEFT_ENC_CLK | D18 | Input + External Interrupt | Quadrature channel A |
| Left Encoder | LEFT_ENC_DT | D19 | Input + External Interrupt | Quadrature channel B |
| Right Encoder | RIGHT_ENC_CLK | D20 | Input + External Interrupt | Quadrature channel A |
| Right Encoder | RIGHT_ENC_DT | D21 | Input + External Interrupt | Quadrature channel B |
| Sonar | SONAR_TRIG_PIN | D39 | Output | Trigger pulse |
| Sonar | SONAR_ECHO_PIN | D41 | Input | Echo pulse; sensor powered from the 5V/GND rails |
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
- Current external interrupt mapping on Mega supports these pins:
  - D18, D19, D20, D21

### Sonar

- Sonar stop threshold: 20 cm.
- When a valid sonar reading is below the threshold, firmware commands both motors to zero.
- D37 powers sensor VCC HIGH and D43 provides sensor GND LOW.

### BMI270 I2C

- The main motor-control firmware now reserves D20/D21 for the right encoder, so BMI270 I2C wiring on D20/D21 applies only to the standalone BMI270 test firmware or to alternate wiring/builds.
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

- Used digital pins: D0, D1, D4, D5, D6, D7, D13, D18, D19, D20, D21, D37, D39, D41, D43
- Unused digital pins: all others (available for future expansion)
- Analog pins A0-A15: currently unused

## Power and Ground Guidance

- Use a common ground between Arduino, motor driver, encoders, and IMU.
- Power motors from a suitable motor supply through TB6612FNG VM.
- Power TB6612 logic according to driver voltage requirements; power the BMI270 module from 3.3V.
- Do not power motors directly from Arduino 5V rail.

## Change Control

If any mapping changes in include/motor_interface.h, update this document and reflash firmware to keep wiring and software aligned.
