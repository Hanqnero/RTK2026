#include <Arduino.h>
#include <stdint.h>

#include <GyverMotor2.h>

#include "control_protocol.h"
#include "encoder.h"
#include "motor_interface.h"

namespace {

struct MotorStep {
    const char* name;
    int16_t left_pwm;
    int16_t right_pwm;
    uint32_t duration_ms;
};

constexpr MotorStep kTestSteps[] = {
    {"left_forward", 20, 0, 2000},
    {"left_reverse", -20, 0, 2000},
    {"right_forward", 0, 20, 2000},
    {"right_reverse", 0, -20, 2000},
    {"both_forward", 20, 20, 2000},
    {"both_reverse", -20, -20, 2000},
};

constexpr size_t kTestStepCount = sizeof(kTestSteps) / sizeof(kTestSteps[0]);

GyverMotor2<GM2::DIR_DIR_PWM> left_motor(LEFT_AI1, LEFT_AI2, LEFT_PWMA);
GyverMotor2<GM2::DIR_DIR_PWM> right_motor(RIGHT_BI1, RIGHT_BI2, RIGHT_PWMB);
EncoderCounter left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, kLeftEncoderReverse, INPUT_PULLUP);
EncoderCounter right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, kRightEncoderReverse, INPUT_PULLUP);
TelemetryPacket telemetry_packet = {};

uint32_t last_step_ms = 0;
size_t step_index = 0;
bool status_led_state = false;

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

void applyStep(size_t index) {
    const MotorStep& step = kTestSteps[index];
    left_motor.runSpeed(step.left_pwm);
    right_motor.runSpeed(step.right_pwm);
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);
    Serial.setTimeout(5);

    configureHardware();

    left_motor.runSpeed(0);
    right_motor.runSpeed(0);

    last_step_ms = millis();
    applyStep(step_index);
}

void loop() {
    const uint32_t now = millis();
    const MotorStep& step = kTestSteps[step_index];

    if (now - last_step_ms >= step.duration_ms) {
        step_index = (step_index + 1U) % kTestStepCount;
        last_step_ms = now;
        applyStep(step_index);
    }

    status_led_state = !status_led_state;
    digitalWrite(LED_BUILTIN, status_led_state ? HIGH : LOW);

    writeTelemetry(kTestSteps[step_index].left_pwm, kTestSteps[step_index].right_pwm);
}
