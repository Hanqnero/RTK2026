#pragma once

#include <stdint.h>

// TB6612FNG pin mapping.
// Motor A (left): AI1, AI2, PWMA
// Motor B (right): BI1, BI2, PWMB
// BI1 and BI2 are direction selects and must not be HIGH at the same time.
// Update these pins to match the actual Arduino Mega wiring.
#define LEFT_AI1 8
#define LEFT_AI2 9
#define LEFT_PWMA 12

#define RIGHT_BI1 10
#define RIGHT_BI2 11
#define RIGHT_PWMB 13

#define LEFT_ENC_CLK 2
#define LEFT_ENC_DT 3
#define RIGHT_ENC_CLK 4
#define RIGHT_ENC_DT 5

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
