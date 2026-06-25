#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include <GyverMotor2.h>
#include <uPID.h>

#include "control_protocol.h"
#include "encoder.h"
#include "motor_interface.h"
#include "sonar.h"

namespace {

constexpr float kWheelCircumferenceM = 2.0F * kPi * kWheelRadiusM;
constexpr float kControlPeriodS = static_cast<float>(kControlPeriodMs) / 1000.0f;

GyverMotor2<GM2::PWM_PWM_SPEED> left_motor(LEFT_PWM_A, LEFT_PWM_B);
GyverMotor2<GM2::PWM_PWM_SPEED> right_motor(RIGHT_PWM_A, RIGHT_PWM_B);

EncoderCounter left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, kLeftEncoderReverse, INPUT_PULLUP);
EncoderCounter right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, kRightEncoderReverse, INPUT_PULLUP);

uPID linear_pid( D_ERROR, kControlPeriodMs);
uPID angular_pid( D_ERROR, kControlPeriodMs);

ControlPacket command_packet = {0.0F, 0.0F, 0U};
TelemetryPacket telemetry_packet = {};
bool debug_raw_encoder = false;

uint32_t last_control_ms = 0;
uint32_t last_command_ms = 0;

float odom_x_m = 0.0f;
float odom_y_m = 0.0f;
float odom_heading_rad = 0.0f;
bool status_led_state = false;

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

struct EncoderDeltas {
    int32_t left;
    int32_t right;
};

int32_t absInt32(int32_t value) {
    return value < 0 ? -value : value;
}

EncoderDeltas filterEncoderDeltas(
    int32_t left_delta,
    int32_t right_delta,
    float target_linear_mps,
    float target_angular_rps) {
    const bool linear_stopped = fabsf(target_linear_mps) <= kLinearCommandDeadbandMps;
    const bool angular_stopped = fabsf(target_angular_rps) <= kAngularCommandDeadbandRadS;

    if (linear_stopped && angular_stopped &&
        absInt32(left_delta) <= kStoppedWheelDeltaDeadbandCounts &&
        absInt32(right_delta) <= kStoppedWheelDeltaDeadbandCounts) {
        return {0, 0};
    }

    if (!linear_stopped && angular_stopped &&
        absInt32(right_delta - left_delta) <= kStraightDiffDeltaDeadbandCounts) {
        const int32_t average_delta = (left_delta + right_delta) / 2;
        return {average_delta, average_delta};
    }

    return {left_delta, right_delta};
}

float countsToMotorRps(int32_t counts_per_cycle) {
    const float counts_per_second = counts_per_cycle * (1000.0f / kControlPeriodMs);
    return counts_per_second / kEncoderCountsPerMotorRev;
}

float motorRpsToRpm(float motor_rps) {
    return motor_rps * 60.0f;
}

float motorRpsToWheelLinearMps(float motor_rps) {
    const float wheel_rps = motor_rps;
    return wheel_rps * kWheelCircumferenceM;
}

float linearMpsToWheelRpm(float linear_mps) {
    return (linear_mps / kWheelCircumferenceM) * 60.0f;
}

float angularRadSToWheelDeltaRpm(float angular_rad_s) {
    return (angular_rad_s * kTrackWidthM / kWheelCircumferenceM) * 60.0f;
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
    configurePid(linear_pid, kLMotorKp, kLMotorKi, kLMotorKd, -kMaxPwmCommand, kMaxPwmCommand);
    configurePid(angular_pid, kAMotorKp, kAMotorKi, kAMotorKd, -kMaxPwmCommand, kMaxPwmCommand);
}

void configureHardware() {
    left_motor.setReverse(kLeftMotorReverse);
    right_motor.setReverse(kRightMotorReverse);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    configureSonar();

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
        debug_raw_encoder = command_packet.debug_raw_encoder != 0;
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
    const EncoderDeltas filtered_deltas = filterEncoderDeltas(
        left_delta,
        right_delta,
        command_packet.target_linear_mps,
        command_packet.target_angular_rps);

    const float left_motor_rps = countsToMotorRps(filtered_deltas.left);
    const float right_motor_rps = countsToMotorRps(filtered_deltas.right);
    const float current_left_wheel_rpm = motorRpsToRpm(left_motor_rps);
    const float current_right_wheel_rpm = motorRpsToRpm(right_motor_rps);

    const float current_left_wheel_mps = motorRpsToWheelLinearMps(left_motor_rps);
    const float current_right_wheel_mps = motorRpsToWheelLinearMps(right_motor_rps);

    const float current_linear_mps = 0.5f * (current_left_wheel_mps + current_right_wheel_mps);
    const float current_angular_rps = -(current_right_wheel_mps - current_left_wheel_mps) / kTrackWidthM;
    const float current_linear_rpm = 0.5f * (current_left_wheel_rpm + current_right_wheel_rpm);
    const float current_angular_rpm = -(current_right_wheel_rpm - current_left_wheel_rpm);

    // Midpoint integration improves accuracy versus pure Euler for curved motion.
    const float delta_heading = current_angular_rps * kControlPeriodS;
    const float heading_mid = odom_heading_rad + 0.5f * delta_heading;
    const float delta_s = current_linear_mps * kControlPeriodS;
    odom_x_m += delta_s * cosf(heading_mid);
    odom_y_m += delta_s * sinf(heading_mid);
    odom_heading_rad = wrapAngleRad(odom_heading_rad + delta_heading);

    if (sonarObstacleDetected()) {
        command_packet.target_linear_mps = 0.0f;
        command_packet.target_angular_rps = 0.0f;
        left_motor.runSpeed(0);
        right_motor.runSpeed(0);

        telemetry_packet.odom_x_m = odom_x_m;
        telemetry_packet.odom_y_m = odom_y_m;
        telemetry_packet.odom_heading_rad = odom_heading_rad;
        telemetry_packet.raw_left_encoder_delta = debug_raw_encoder ? left_delta : 0;
        telemetry_packet.raw_right_encoder_delta = debug_raw_encoder ? right_delta : 0;
        telemetry_packet.left_pwm = 0;
        telemetry_packet.right_pwm = 0;
        telemetry_packet.current_linear_mps = current_linear_mps;
        telemetry_packet.current_angular_rps = current_angular_rps;

        if (Serial.availableForWrite() >= static_cast<int>(sizeof(TelemetryPacket))) {
            Serial.write(reinterpret_cast<const uint8_t*>(&telemetry_packet), sizeof(TelemetryPacket));
        }
        return;
    }

    linear_pid.setpoint = linearMpsToWheelRpm(command_packet.target_linear_mps);
    angular_pid.setpoint = angularRadSToWheelDeltaRpm(command_packet.target_angular_rps);

    const float linear_pwm = linear_pid.compute(current_linear_rpm);
    const float angular_pwm = angular_pid.compute(current_angular_rpm);

    const int16_t left_pwm =
        static_cast<int16_t>(clampFloat(linear_pwm - angular_pwm, -kMaxPwmCommand, kMaxPwmCommand));
    const int16_t right_pwm =
        static_cast<int16_t>(clampFloat(linear_pwm + angular_pwm, -kMaxPwmCommand, kMaxPwmCommand));
    const int16_t left_pwm_limited =
        static_cast<int16_t>(clampFloat(static_cast<float>(left_pwm), -kMaxPwmCommand, kMaxPwmCommand));
    const int16_t right_pwm_limited =
        static_cast<int16_t>(clampFloat(static_cast<float>(right_pwm), -kMaxPwmCommand, kMaxPwmCommand));

    left_motor.runSpeed(left_pwm_limited);
    right_motor.runSpeed(right_pwm_limited);

    status_led_state = !status_led_state;
    digitalWrite(LED_BUILTIN, status_led_state ? HIGH : LOW);

    telemetry_packet.odom_x_m = odom_x_m;
    telemetry_packet.odom_y_m = odom_y_m;
    telemetry_packet.odom_heading_rad = odom_heading_rad;
    telemetry_packet.raw_left_encoder_delta = debug_raw_encoder ? left_delta : 0;
    telemetry_packet.raw_right_encoder_delta = debug_raw_encoder ? right_delta : 0;
    telemetry_packet.left_pwm = left_pwm_limited;
    telemetry_packet.right_pwm = right_pwm_limited;
    telemetry_packet.current_linear_mps = current_linear_mps;
    telemetry_packet.current_angular_rps = current_angular_rps;

    if (Serial.availableForWrite() >= static_cast<int>(sizeof(TelemetryPacket))) {
        Serial.write(reinterpret_cast<const uint8_t*>(&telemetry_packet), sizeof(TelemetryPacket));
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);
    Serial.setTimeout(5);

    configureHardware();
    configurePids();

    left_motor.runSpeed(0);
    right_motor.runSpeed(0);

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
