#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include <GyverMotor2.h>

#include "encoder.h"
#include "motor_interface.h"

namespace {

constexpr int16_t kTestPwm = 63;
constexpr uint16_t kStepDurationMs = 2000;
constexpr uint16_t kPrintPeriodMs = 500;

struct TestStep {
    int16_t left_pwm;
    int16_t right_pwm;
};

constexpr TestStep kTestSteps[] = {
    {kTestPwm, kTestPwm},
    {-kTestPwm, -kTestPwm},
    {kTestPwm, -kTestPwm},
    {-kTestPwm, kTestPwm},
    {0, 0},
};

GyverMotor2<GM2::PWM_PWM_SPEED> left_motor(LEFT_PWM_A, LEFT_PWM_B);
GyverMotor2<GM2::PWM_PWM_SPEED> right_motor(RIGHT_PWM_A, RIGHT_PWM_B);
EncoderCounter left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, kLeftEncoderReverse, INPUT_PULLUP);
EncoderCounter right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, kRightEncoderReverse, INPUT_PULLUP);

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

void printEncoderSample() {
    const int32_t left_count = left_encoder.readCount();
    const int32_t right_count = right_encoder.readCount();

    Serial.print("left count: ");
    Serial.print(left_count);
    Serial.print(" \t ");
    Serial.print("right count: ");
    Serial.println(right_count);
}

}  // namespace



void setup() {
    Serial.begin(kSerialBaudRate);
    Serial.setTimeout(100);

    configureHardware();

    left_motor.runSpeed(100);
    right_motor.runSpeed(100);
}

void printLong(unsigned long l) {
    char buffer[11];  // 10 digits + null terminator
    snprintf(buffer, sizeof(buffer), "%lu", l);
    Serial.print(buffer);
}

void loop() {
    printEncoderSample();
}
