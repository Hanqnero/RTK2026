This is an automatic translation, may be incorrect in some places. See sources and examples!

# GyverMotor2
Library for controlling commutator motors via a driver, an improved version of the library [GyverMotor](https://github.com/GyverLibs/GyverMotor)
- Control of speed and direction of rotation
- Work with PWM of any resolution
- Smooth start and speed change
- Minimum PWM threshold
- Software deadtime
- Supports 7 types of drivers

### Compatibility
Compatible with all Arduino platforms (uses Arduino functions)

## Contents
- [Usage](#usage)
- [Versions](#versions)
- [Installation](#install)
- [Bugs and feedback](#feedback)

<a id="usage"></a>

## Usage
### Driver type
|Driver |Description |Pins |speed > 0 |speed < 0 |stop |brake |
|---------------------|------------------------|-------------------|-------------|-------------|-----------|-------------|
|`GM2::ONLY_PWM` |PWM only |(PWM) |(PWM) |(PWM) |(0) |(0) |
|`GM2::DIR_DIR` |Direction only |(GPIO, GPIO) |(0, 1) |(1, 0) |(0, 0) |(1, 1) |
|`GM2::DIR_PWM` |One PWM without inversion |(GPIO, PWM) |(0, PWM) |(1, PWM) |(0, 0) |(1, MAX) |
|`GM2::DIR_PWM_INV` |One PWM with inversion |(GPIO, PWM) |(0, PWM) |(1, ~PWM) |(0, 0) |(1, MAX) |
|`GM2::PWM_PWM_SPEED` |Two PWM, speed mode |(PWM, PWM) |(0, PWM) |(PWM, 0) |(0, 0) |(MAX, MAX) |
|`GM2::PWM_PWM_POWER` |Two PWM, torque mode |(PWM, PWM) |(~PWM, MAX) |(MAX, ~PWM) |(0, 0) |(MAX, MAX) |
|`GM2::DIR_DIR_PWM` |Two directions and PWM |(GPIO, GPIO, PWM) |(0, 1, PWM) |(1, 0, PWM) |(0, 0, 0) |(1, 1, MAX) |

Notes:

- The pins in parentheses are in the order of the constructor, i.e.for example, for `DIR_PWM` the initialization will be `GyverMotor2<GM2::DIR_PWM> motor(gpio, pwm)`, where gpio is a regular digital pin (digitalWrite), pwm is a pin with PWM support (analogWrite)
- `MAX` - maximum PWM value at the specified resolution
- `PWM` - direct PWM `(0.. MAX)`
- `~PWM` - reverse PWM `(MAX.. 0)`

Mode selection for H-bridge driver (2 pins, usually IN1 and IN2) - most driver modules for Arduino:

- For symmetrical speed operation in both directions, a dual PWM mode is recommended, where `PWM_PWM_SPEED` has a higher speed, and `PWM_PWM_POWER` is more stable at low speeds and has a higher torque on some motors and drivers.Ideal for wheeled robots
- To work in both directions, when the symmetry of the motor operation is not important, you can use the `DIR_PWM_INV` mode - it saves one PWM pin.Drivers that work with `DIR_PWM` (without inversion) are rare
- For one-way operation with speed control (MOSFET or bridge with a closed pin), the `ONLY_PWM` mode is suitable and uses only one pin
- For relay control (without speed control) you can use the `DIR_DIR` mode

For drivers with separate direction controlI eat cranberries and the speed IN1+IN2+EN needs the `DIR_DIR_PWM` mode.

### Compilation settings
Before connecting the library

```cpp
#define GMOTOR2_NO_ACCEL // cut acceleration module
```

### Class description
```cpp
// initialization indicating driver type, PWM resolution (bits) and pins
GyverMotor2<driver, res = 8>(uint8_t pinA, uint8_t pinB = 0xff, uint8_t pinC = 0xff);

// set deadtime (delay when changing direction) in microseconds (default 0)
void setDeadtime(uint8_t us);

// get deadtime in microseconds
uint8_t getDeadTime();

// set direction reverse (default false)
void setReverse(bool rev);

// get direction reverse
bool getReverse();

// set minimum PWM (default 0)
void setMinDuty(uint16_t duty);

// set the minimum PWM as a % of the maximum (default 0)
void setMinDutyPerc(uint8_t perc);

// get minimum PWM
uint16_t getMinDuty();

// get maximum PWM
uint16_t getMaxDuty();

// set PWM speed (-max.. max)
void runSpeed(int16_t speed);

// set the speed as a % of the maximum PWM (-100.. 100%)
void runSpeedPerc(int8_t perc);

// get the current motor speed in PWM
int16_t getSpeed();

// get direction: motor is spinning (1 and -1), motor is stopped (0)
int8_t getDir();

// stop.If acceleration is enabled, then smooth
void stop();

// active braking (keeps driver pins active)
void brake();

// stop and turn off the motor
void disable();

// set acceleration in PWM per second.The real minimum is 5.0 to disable
void setAccel(uint16_t accel);

// set the acceleration in percent per second.0 to disable
void setAccelPerc(uint8_t perc);

// get the period at least which tick should be called, ms
uint8_t getDt();

// get target speed (acceleration mode)
int16_t getTarget();

// smooth change to the specified speed with setAccel acceleration, call in loop.Returns true when speed is reached
bool tick();
```

### Description of features
#### Start and stop
- `runSpeed` starts the motor at the specified speed, supports negative values for rotation in the opposite direction.When set to `0`, turns off the motor
- `stop` - stop and shutdown, equivalent to calling `runSpeed(0)`
- `setReverse` - motor reverse, when set to `true` it will rotate in the opposite direction
- `brake` - active braking via the driver (not supported everywhere)
- `disable` - stop and disable the driver

#### DeadTime
Inside, `runSpeed`, when changing the direction of rotation, turns off the driver and creates a delay for the specified number of microseconds, after which it sends the required signal to the motor.This is necessary for homemade simple H-bridge drivers to avoid shorting the bridge.Drivers in the form of chips and modules usually have built-in hardware deadtime and do not need to be enabled in the library.

#### MinDuty
Allows you to set the minimum signal to the motor.The value supplied to `runSpeed` will scale linearly from the specified MinDuty to the maximum (if not zero).This allows you to “skip” the values ​​at which the motor cannot move without changing the logic of the speed setting.Example values for 8 bits and `MinDuty=50`:

|`runSpeed` |Actual PWM |
|------------|-----------------|
|0 |0 |
|1 |50 |
|50 |90 |
|150 |170 |
|200 |210 |
|255 |255 |

#### Accel
When specifying a non-zero acceleration (in units of PWM per second), the `runSpeed` call will not apply the specified speed immediately, but will instead ramp smoothly towards the specified speed.To work, you need to call the ticker in the main program loop no less often than `getDt` milliseconds (varies from 30 to 200 depending on the acceleration).Calling `stop` will make a smooth stop, while `brake` and `disable` will stop the motor abruptly:

```cpp
void setup() {
motor.setAccel(80);
motor.runSpeed(170);
}

void loop() {
motor.tick();// smoothly accelerates the motor to speed 170
}
```

Might come in handyis used when controlling a heavy wheeled robot to avoid jerks during acceleration and braking.

## Examples
```cpp
#include <GyverMotor2.h>

// GyverMotor2<GM2::DIR_DIR> motor(5, 6);
// GyverMotor2<GM2::DIR_PWM> motor(5, 6);
GyverMotor2<GM2::DIR_PWM_INV> motor(5, 6);
// GyverMotor2<GM2::PWM_PWM_SPEED> motor(5, 6);
// GyverMotor2<GM2::PWM_PWM_POWER> motor(5, 6);
// GyverMotor2<GM2::DIR_DIR_PWM> motor(5, 6, 7);

void setup() {
// motor.setDeadtime(5);
// motor.setMinDuty(50);
// motor.setReverse(true);
}

void loop() {
int speed = map(analogRead(0), 0, 1023, -255, 255);
motor.runSpeed(speed);
}
```

<a id="versions"></a>

## Versions
- v1.0

<a id="install"></a>
## Installation
- The library can be found by the name **GyverMotor2** and installed through the library manager in:
- Arduino IDE
- Arduino IDE v2
- PlatformIO
- [Download library](https://github.com/GyverLibs/GyverMotor2/archive/refs/heads/main.zip).zip archive for manual installation:
- Unpack and put in *C:\Program Files (x86)\Arduino\libraries* (Windows x64)
- Unpack and put in *C:\Program Files\Arduino\libraries* (Windows x32)
- Unpack and put in *Documents/Arduino/libraries/*
- (Arduino IDE) automatic installation from .zip: *Sketch/Connect library/Add .ZIP library…* and indicate the downloaded archive
- Read more detailed instructions for installing libraries[here](https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)
### Update
- I recommend always updating the library: in new versions errors and bugs are corrected, as well as optimization is carried out and new features are added
- Through the IDE library manager: find the library as during installation and click "Update"
- Manually: **delete the folder with the old version**, and then put the new one in its place.“Replacement” cannot be done: sometimes new versions delete files that will remain after replacement and can lead to errors!

<a id="feedback"></a>

## Bugs and feedback
When you find bugs, create an **Issue**, or better yet, immediately write to [alex@alexgyver.ru](mailto:alex@alexgyver.ru)
The library is open for improvement and your **Pull Requests**!

When reporting bugs or incorrect operation of the library, be sure to indicate:
- Library version
- Which MK is used?
- SDK version (for ESP)
- Arduino IDE version
- Do the built-in examples that use functions and constructs that lead to a bug in your code work correctly?
- What code was loaded, what work was expected from it and how it works in reality
- Ideally, attach the minimum code in which the bug is observed.Not a canvas of a thousand lines, but minimal code