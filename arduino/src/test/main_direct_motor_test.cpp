#include <Arduino.h>
#include <stdint.h>

#include <GyverMotor2.h>

#include "encoder.h"
#include "motor_interface.h"

namespace {

constexpr int16_t kTestPwm = 80;
constexpr uint16_t kDriveStepDurationMs = 2000;
constexpr uint16_t kStopStepDurationMs = 700;
constexpr uint16_t kPrintPeriodMs = 100;

struct TestStep {
    const char* label;
    int16_t left_pwm;
    int16_t right_pwm;
    uint16_t duration_ms;
};

constexpr TestStep kTestSteps[] = {
    {"idle", 0, 0, kStopStepDurationMs},
    {"left_pos", kTestPwm, 0, kDriveStepDurationMs},
    {"stop", 0, 0, kStopStepDurationMs},
    {"left_neg", -kTestPwm, 0, kDriveStepDurationMs},
    {"stop", 0, 0, kStopStepDurationMs},
    {"right_pos", 0, kTestPwm, kDriveStepDurationMs},
    {"stop", 0, 0, kStopStepDurationMs},
    {"right_neg", 0, -kTestPwm, kDriveStepDurationMs},
    {"stop", 0, 0, kStopStepDurationMs},
    {"both_pos", kTestPwm, kTestPwm, kDriveStepDurationMs},
    {"stop", 0, 0, kStopStepDurationMs},
    {"both_neg", -kTestPwm, -kTestPwm, kDriveStepDurationMs},
    {"done", 0, 0, kStopStepDurationMs},
};
constexpr uint8_t kTestStepCount = sizeof(kTestSteps) / sizeof(kTestSteps[0]);

GyverMotor2<GM2::PWM_PWM_SPEED> left_motor(LEFT_PWM_A, LEFT_PWM_B);
GyverMotor2<GM2::PWM_PWM_SPEED> right_motor(RIGHT_PWM_A, RIGHT_PWM_B);
EncoderCounter left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, kLeftEncoderReverse, INPUT_PULLUP);
EncoderCounter right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, kRightEncoderReverse, INPUT_PULLUP);

uint8_t step_index = 0;
uint32_t step_start_ms = 0;
uint32_t last_print_ms = 0;
int32_t total_left_delta = 0;
int32_t total_right_delta = 0;

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

void applyStep(const TestStep& step) {
    left_motor.runSpeed(step.left_pwm);
    right_motor.runSpeed(step.right_pwm);
    left_encoder.readAndResetDelta();
    right_encoder.readAndResetDelta();
    total_left_delta = 0;
    total_right_delta = 0;
}

void printHeader() {
    Serial.println(F("timestamp_ms,step_index,phase,phase_elapsed_ms,left_pwm,right_pwm,left_delta,right_delta,left_total_delta,right_total_delta"));
}

void printEncoderSample(uint32_t now) {
    const TestStep& step = kTestSteps[step_index];
    const int32_t left_delta = left_encoder.readAndResetDelta();
    const int32_t right_delta = right_encoder.readAndResetDelta();
    total_left_delta += left_delta;
    total_right_delta += right_delta;

    Serial.print(now);
    Serial.print(',');
    Serial.print(step_index);
    Serial.print(',');
    Serial.print(step.label);
    Serial.print(',');
    Serial.print(now - step_start_ms);
    Serial.print(',');
    Serial.print(step.left_pwm);
    Serial.print(',');
    Serial.print(step.right_pwm);
    Serial.print(',');
    Serial.print(left_delta);
    Serial.print(',');
    Serial.print(right_delta);
    Serial.print(',');
    Serial.print(total_left_delta);
    Serial.print(',');
    Serial.println(total_right_delta);
}

void advanceStep(uint32_t now) {
    if (step_index + 1 < kTestStepCount) {
        ++step_index;
    }

    step_start_ms = now;
    last_print_ms = now;
    applyStep(kTestSteps[step_index]);
    printEncoderSample(now);
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);
    Serial.setTimeout(100);

    configureHardware();

    step_start_ms = millis();
    last_print_ms = step_start_ms;
    applyStep(kTestSteps[step_index]);
    printHeader();
    printEncoderSample(step_start_ms);
}

void loop() {
    const uint32_t now = millis();
    const TestStep& step = kTestSteps[step_index];

    if ((uint32_t)(now - step_start_ms) >= step.duration_ms) {
        advanceStep(now);
        return;
    }

    if ((uint32_t)(now - last_print_ms) >= kPrintPeriodMs) {
        last_print_ms += kPrintPeriodMs;
        printEncoderSample(now);
    }
}
