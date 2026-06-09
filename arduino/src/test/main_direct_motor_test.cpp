#include <Arduino.h>
#include <stdint.h>

#include <GyverMotor2.h>

#include "control_protocol.h"
#include "encoder.h"
#include "motor_interface.h"
#include "sonar.h"

namespace {

constexpr int16_t kTestPwm = static_cast<int16_t>(255.0f * 0.80f);
constexpr char kStopCommand[] = "stop";

GyverMotor2<GM2::PWM_PWM_SPEED> left_motor(LEFT_PWM_A, LEFT_PWM_B);
GyverMotor2<GM2::PWM_PWM_SPEED> right_motor(RIGHT_PWM_A, RIGHT_PWM_B);
EncoderCounter left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, kLeftEncoderReverse, INPUT_PULLUP);
EncoderCounter right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, kRightEncoderReverse, INPUT_PULLUP);
TelemetryPacket telemetry_packet = {};

bool status_led_state = false;
bool motors_started = false;
uint8_t stop_match_index = 0;

void leftEncoderISR() {
    left_encoder.handleInterrupt();
}

void rightEncoderISR() {
    right_encoder.handleInterrupt();
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

void writeTelemetry(int16_t left_pwm, int16_t right_pwm) {
    telemetry_packet.odom_x_m = 0.0f;
    telemetry_packet.odom_y_m = 0.0f;
    telemetry_packet.odom_heading_rad = 0.0f;
    telemetry_packet.raw_left_encoder_delta = left_encoder.readAndResetDelta();
    telemetry_packet.raw_right_encoder_delta = right_encoder.readAndResetDelta();
    telemetry_packet.left_pwm = left_pwm;
    telemetry_packet.right_pwm = right_pwm;

    if (Serial.availableForWrite() >= static_cast<int>(sizeof(TelemetryPacket))) {
        Serial.write(reinterpret_cast<const uint8_t*>(&telemetry_packet), sizeof(TelemetryPacket));
    }
}

void stopMotors() {
    motors_started = false;
    left_motor.runSpeed(0);
    right_motor.runSpeed(0);
}

void startMotors() {
    motors_started = true;
    left_motor.runSpeed(kTestPwm);
    right_motor.runSpeed(kTestPwm);
}

void readSerialCommands() {
    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());
        if (ch == kStopCommand[stop_match_index]) {
            ++stop_match_index;
            if (kStopCommand[stop_match_index] == '\0') {
                stopMotors();
                stop_match_index = 0;
            }
        } else {
            stop_match_index = (ch == kStopCommand[0]) ? 1U : 0U;
        }
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);
    Serial.setTimeout(5);

    configureHardware();

    startMotors();
}

void loop() {
    readSerialCommands();

    if (motors_started && sonarObstacleDetected()) {
        stopMotors();
    }

    status_led_state = !status_led_state;
    digitalWrite(LED_BUILTIN, status_led_state ? HIGH : LOW);

    writeTelemetry(motors_started ? kTestPwm : 0, motors_started ? kTestPwm : 0);
}
