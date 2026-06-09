#pragma once

#include <stdint.h>

// Dual-PWM motor driver pin mapping.
// Motor A (left): PWM A, PWM B
// Motor B (right): PWM A, PWM B
// Update these pins to match the actual Arduino Mega wiring.
#define LEFT_PWM_A 7
#define LEFT_PWM_B 6

#define RIGHT_PWM_A 5
#define RIGHT_PWM_B 4

#define LEFT_ENC_CLK 18
#define LEFT_ENC_DT 19
#define RIGHT_ENC_CLK 20
#define RIGHT_ENC_DT 21

#define SONAR_VCC_PIN 37
#define SONAR_TRIG_PIN 39
#define SONAR_ECHO_PIN 41
#define SONAR_GND_PIN 43

constexpr bool kLeftMotorReverse = false;
constexpr bool kRightMotorReverse = false;
constexpr bool kLeftEncoderReverse = false;
constexpr bool kRightEncoderReverse = true;

constexpr uint8_t kImuSpiCsPin = 53;
constexpr uint32_t kImuSpiClockHz = 1000000UL;
constexpr float kImuGyroDpsPerLsb = 500.0f / 32768.0f;
constexpr float kOdomYawRateEncoderWeight = 0.7f;
constexpr uint16_t kImuGyroBiasSampleCount = 50;

constexpr uint32_t kSerialBaudRate = 115200;
constexpr uint16_t kControlPeriodMs = 100;
constexpr uint16_t kCommandTimeoutMs = 500;

constexpr float kWheelRadiusM = 0.06f;
constexpr float kTrackWidthM = 0.30f;

// Effective encoder counts per motor revolution after quadrature decoding.
constexpr float kEncoderCountsPerMotorRev = 1024.0f;

// The requested spec says "gearbox ratio is 10:0"; interpret that as 10.0:1.
// Set to 1.0 if the encoder is mounted on the wheel/output shaft instead.
constexpr float kGearboxRatio = 10.0f;
constexpr float kMaxMotorRpm = 960.0f;

constexpr float kMaxLinearCommandMps = 1.5f;
constexpr float kMaxAngularCommandRadS = 6.0f;

constexpr float kMaxLinearCorrectionMps = 1.0f;
constexpr float kMaxAngularCorrectionRadS = 4.0f;

constexpr float kLinearKp = 0.6f;
constexpr float kLinearKi = 0.2f;
constexpr float kLinearKd = 0.0f;

constexpr float kAngularKp = 0.8f;
constexpr float kAngularKi = 0.15f;
constexpr float kAngularKd = 0.0f;

constexpr float kMotorKp = 220.0f;
constexpr float kMotorKi = 40.0f;
constexpr float kMotorKd = 0.0f;

constexpr float kMaxPwmDuty = 0.90f;
constexpr float kMaxPwmCommand = 255.0f * kMaxPwmDuty;

constexpr float kSonarStopThresholdCm = 20.0f;
constexpr uint32_t kSonarEchoTimeoutUs = 25000UL;
constexpr uint16_t kSonarSamplePeriodMs = 60;
