#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include <GyverMotor2.h>
#include <uPID.h>

#include "control_protocol.h"
#include "encoder.h"
#include "motor_interface.h"

namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kWheelCircumferenceM = 2.0F * kPi * kWheelRadiusM;
constexpr float kControlPeriodS = static_cast<float>(kControlPeriodMs) / 1000.0f;

GyverMotor2<GM2::DIR_DIR_PWM> left_motor(LEFT_AI1, LEFT_AI2, LEFT_PWMA);
GyverMotor2<GM2::DIR_DIR_PWM> right_motor(RIGHT_BI1, RIGHT_BI2, RIGHT_PWMB);

EncoderCounter left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, kLeftEncoderReverse, INPUT_PULLUP);
EncoderCounter right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, kRightEncoderReverse, INPUT_PULLUP);

uPID linear_pid(I_SATURATE, kControlPeriodMs);
uPID angular_pid(I_SATURATE, kControlPeriodMs);
uPID left_motor_pid(I_SATURATE | D_INPUT, kControlPeriodMs);
uPID right_motor_pid(I_SATURATE | D_INPUT, kControlPeriodMs);

ControlPacket command_packet = {0.0F, 0.0F};
TelemetryPacket telemetry_packet = {};

uint32_t last_control_ms = 0;
uint32_t last_command_ms = 0;

float odom_x_m = 0.0f;
float odom_y_m = 0.0f;
float odom_heading_rad = 0.0f;

void leftEncoderISR() {
    left_encoder.handleInterrupt();
}

void rightEncoderISR() {
    right_encoder.handleInterrupt();
}

float clampFloat(float value, float low, float high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

float countsToMotorRps(int32_t counts_per_cycle) {
    const float counts_per_second = counts_per_cycle * (1000.0f / kControlPeriodMs);
    return counts_per_second / kEncoderCountsPerMotorRev;
}

float motorRpsToWheelLinearMps(float motor_rps) {
    const float wheel_rps = motor_rps / kGearboxRatio;
    return wheel_rps * kWheelCircumferenceM;
}

float wrapAngleRad(float angle) {
    while (angle > kPi) {
        angle -= 2.0f * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0f * kPi;
    }
    return angle;
}

void configurePid(uPID& pid, float kp, float ki, float kd, float out_min, float out_max) {
    pid.setKp(kp);
    pid.setKi(ki);
    pid.setKd(kd);
    pid.outMin = out_min;
    pid.outMax = out_max;
}

void configurePids() {
    configurePid(linear_pid, kLinearKp, kLinearKi, kLinearKd, -kMaxLinearCorrectionMps, kMaxLinearCorrectionMps);
    configurePid(angular_pid, kAngularKp, kAngularKi, kAngularKd, -kMaxAngularCorrectionRadS, kMaxAngularCorrectionRadS);
    configurePid(left_motor_pid, kMotorKp, kMotorKi, kMotorKd, -255.0f, 255.0f);
    configurePid(right_motor_pid, kMotorKp, kMotorKi, kMotorKd, -255.0f, 255.0f);
}

void configureHardware() {
    left_motor.setReverse(kLeftMotorReverse);
    right_motor.setReverse(kRightMotorReverse);

    left_encoder.begin();
    right_encoder.begin();

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_CLK), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_DT), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_CLK), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_DT), rightEncoderISR, CHANGE);
}

void readLatestCommand() {
    while (Serial.available() >= static_cast<int>(sizeof(ControlPacket))) {
        Serial.readBytes(reinterpret_cast<char*>(&command_packet), sizeof(ControlPacket));
        command_packet.target_linear_mps =
            clampFloat(command_packet.target_linear_mps, -kMaxLinearCommandMps, kMaxLinearCommandMps);
        command_packet.target_angular_rps =
            clampFloat(command_packet.target_angular_rps, -kMaxAngularCommandRadS, kMaxAngularCommandRadS);
        last_command_ms = millis();
    }
}

void applyCommandTimeout() {
    if (millis() - last_command_ms > kCommandTimeoutMs) {
        command_packet.target_linear_mps = 0.0f;
        command_packet.target_angular_rps = 0.0f;
    }
}

void runControlCycle() {
    const int32_t left_delta = left_encoder.readAndResetDelta();
    const int32_t right_delta = right_encoder.readAndResetDelta();

    const float left_motor_rps = countsToMotorRps(left_delta);
    const float right_motor_rps = countsToMotorRps(right_delta);

    const float current_left_wheel_mps = motorRpsToWheelLinearMps(left_motor_rps);
    const float current_right_wheel_mps = motorRpsToWheelLinearMps(right_motor_rps);

    const float current_linear_mps = 0.5f * (current_left_wheel_mps + current_right_wheel_mps);
    const float current_angular_rps = (current_right_wheel_mps - current_left_wheel_mps) / kTrackWidthM;

    // Midpoint integration improves accuracy versus pure Euler for curved motion.
    const float delta_heading = current_angular_rps * kControlPeriodS;
    const float heading_mid = odom_heading_rad + 0.5f * delta_heading;
    const float delta_s = current_linear_mps * kControlPeriodS;
    odom_x_m += delta_s * cosf(heading_mid);
    odom_y_m += delta_s * sinf(heading_mid);
    odom_heading_rad = wrapAngleRad(odom_heading_rad + delta_heading);

    linear_pid.setpoint = command_packet.target_linear_mps;
    angular_pid.setpoint = command_packet.target_angular_rps;

    const float linear_error = command_packet.target_linear_mps - current_linear_mps;
    const float angular_error = command_packet.target_angular_rps - current_angular_rps;

    const float linear_correction = linear_pid.compute(current_linear_mps);
    const float angular_correction = angular_pid.compute(current_angular_rps);

    const float corrected_linear_mps = command_packet.target_linear_mps + linear_correction;
    const float corrected_angular_rps = command_packet.target_angular_rps + angular_correction;

    const float target_left_wheel_mps = corrected_linear_mps - corrected_angular_rps * (kTrackWidthM * 0.5f);
    const float target_right_wheel_mps = corrected_linear_mps + corrected_angular_rps * (kTrackWidthM * 0.5f);

    left_motor_pid.setpoint = target_left_wheel_mps;
    right_motor_pid.setpoint = target_right_wheel_mps;

    const float left_motor_error = target_left_wheel_mps - current_left_wheel_mps;
    const float right_motor_error = target_right_wheel_mps - current_right_wheel_mps;

    const int16_t left_pwm = static_cast<int16_t>(left_motor_pid.compute(current_left_wheel_mps));
    const int16_t right_pwm = static_cast<int16_t>(right_motor_pid.compute(current_right_wheel_mps));

    left_motor.runSpeed(left_pwm);
    right_motor.runSpeed(right_pwm);

    telemetry_packet.odom_x_m = odom_x_m;
    telemetry_packet.odom_y_m = odom_y_m;
    telemetry_packet.odom_heading_rad = odom_heading_rad;

    (void)linear_error;
    (void)angular_error;
    (void)left_motor_error;
    (void)right_motor_error;

    Serial.write(reinterpret_cast<const uint8_t*>(&telemetry_packet), sizeof(TelemetryPacket));
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);
    Serial.setTimeout(5);

    configureHardware();
    configurePids();

    last_control_ms = millis();
    last_command_ms = last_control_ms;
}

void loop() {
    readLatestCommand();
    applyCommandTimeout();

    const uint32_t now = millis();
    if (now - last_control_ms < kControlPeriodMs) {
        return;
    }
    last_control_ms += kControlPeriodMs;

    runControlCycle();
}
