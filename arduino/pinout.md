# Arduino Mega Pinout Specification

This document defines the pin mapping used by the current robot firmware on Arduino Mega 2560.

## Scope

- Board: Arduino Mega 2560 (ATmega2560)
- Firmware mapping source: include/motor_interface.h
- Core pin capability source: ArduinoCore-avr-1.8.7/variants/mega/pins_arduino.h

## Pin Assignment Table

| Subsystem | Signal | Arduino Pin | Direction | Notes |
|---|---|---:|---|---|
| Motor Driver (TB6612FNG) | LEFT_AI1 | D8 | Output | Left motor direction pin 1 |
| Motor Driver (TB6612FNG) | LEFT_AI2 | D9 | Output | Left motor direction pin 2 |
| Motor Driver (TB6612FNG) | LEFT_PWMA | D12 | Output (PWM) | Left motor PWM |
| Motor Driver (TB6612FNG) | RIGHT_BI1 | D10 | Output | Right motor direction pin 1 |
| Motor Driver (TB6612FNG) | RIGHT_BI2 | D11 | Output | Right motor direction pin 2 |
| Motor Driver (TB6612FNG) | RIGHT_PWMB | D13 | Output (PWM) | Right motor PWM |
| Left Encoder | LEFT_ENC_CLK | D2 | Input + External Interrupt | Quadrature channel A |
| Left Encoder | LEFT_ENC_DT | D3 | Input + External Interrupt | Quadrature channel B |
| Right Encoder | RIGHT_ENC_CLK | D4 | Input + External Interrupt | Quadrature channel A |
| Right Encoder | RIGHT_ENC_DT | D5 | Input + External Interrupt | Quadrature channel B |
<!-- | BMI270 IMU (SPI) | IMU_CS | D53 | Output | Chip select (SPI SS pin) |
| BMI270 IMU (SPI) | IMU_MOSI | D51 | Output | SPI MOSI (hardware SPI) |
| BMI270 IMU (SPI) | IMU_MISO | D50 | Input | SPI MISO (hardware SPI) |
| BMI270 IMU (SPI) | IMU_SCK | D52 | Output | SPI clock (hardware SPI) | -->
| Host Serial Telemetry/Control | UART0_RX | D0 | Input | USB serial bridge to host TX |
| Host Serial Telemetry/Control | UART0_TX | D1 | Output | USB serial bridge to host RX |

## Interface Requirements

### TB6612FNG

- BI1 and BI2 for a motor must not be driven HIGH at the same time.
- Left motor uses AI1/AI2/PWMA.
- Right motor uses BI1/BI2/PWMB.

### Encoders

- All encoder inputs are configured as INPUT_PULLUP in firmware.
- Current external interrupt mapping on Mega supports these pins:
  - D2, D3, D18, D19

### BMI270 SPI

- SPI bus uses Arduino Mega hardware SPI pins:
  - MISO D50, MOSI D51, SCK D52, SS D53
- IMU CS is assigned to D53.
- SPI clock configured in firmware: 1 MHz.
- Keep D53 configured as output to preserve Mega master SPI mode.

IMU board header mapping (module shown in image, silk labels left->right: `VCC`, `GND`, `SCL`, `SDA`, `SDO`, `CS`, `INT1`, `INT2`):

| IMU board pin | Firmware signal name | Arduino Mega pin | Required |
|---|---|---:|---|
| VCC | IMU power | 3V3 or 5V (module-dependent) | Yes |
| GND | GND | GND | Yes |
| SCL | IMU_SCK | D52 | Yes |
| SDA | IMU_MOSI | D51 | Yes |
| SDO | IMU_MISO | D50 | Yes |
| CS | IMU_CS | D53 | Yes |
| INT1 | IMU_INT1 | Not used by current firmware | No |
| INT2 | IMU_INT2 | Not used by current firmware | No |

Notes:
- On this module, `SCL`/`SDA` are shared-function labels; in SPI mode they map to clock and controller->sensor data respectively.
- `SDO` is sensor->controller data (MISO).
- If interrupts are needed later, connect `INT1` and/or `INT2` to free external-interrupt-capable Mega pins and extend firmware.

### Serial Link

- Telemetry and command packets use Serial (UART0) at 115200 baud.
- USB cable connection uses onboard USB-to-UART bridge mapped to D0/D1.

## Reserved/Used Pins Summary

- Used digital pins: D0, D1, D2, D3, D5, D6, D8, D9, D10, D11, D18, D19, D50, D51, D52, D53
- Unused digital pins: all others (available for future expansion)
- Analog pins A0-A15: currently unused

## Power and Ground Guidance

- Use a common ground between Arduino, motor driver, encoders, and IMU.
- Power motors from a suitable motor supply through TB6612FNG VM.
- Power logic (TB6612 VCC and BMI270 VDD/VDDIO) according to sensor/driver voltage requirements.
- Do not power motors directly from Arduino 5V rail.

## Change Control

If any mapping changes in include/motor_interface.h, update this document and reflash firmware to keep wiring and software aligned.
