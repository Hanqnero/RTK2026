#include "sonar.h"

#include <Arduino.h>

#include "motor_interface.h"

namespace {

uint32_t last_sonar_sample_ms = 0;
bool last_obstacle_detected = false;

float readSonarDistanceCm() {
    digitalWrite(SONAR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG_PIN, LOW);

    const uint32_t echo_us = pulseIn(SONAR_ECHO_PIN, HIGH, kSonarEchoTimeoutUs);
    if (echo_us == 0) {
        return -1.0f;
    }

    return static_cast<float>(echo_us) / 58.0f;
}

}  // namespace

void configureSonar() {
    pinMode(SONAR_VCC_PIN, OUTPUT);
    pinMode(SONAR_GND_PIN, OUTPUT);
    pinMode(SONAR_TRIG_PIN, OUTPUT);
    pinMode(SONAR_ECHO_PIN, INPUT);

    digitalWrite(SONAR_GND_PIN, LOW);
    digitalWrite(SONAR_VCC_PIN, HIGH);
    digitalWrite(SONAR_TRIG_PIN, LOW);
    delay(100);
}

bool sonarObstacleDetectedNow() {
    const float distance_cm = readSonarDistanceCm();
    return distance_cm > 0.0f && distance_cm < kSonarStopThresholdCm;
}

bool sonarObstacleDetected() {
    const uint32_t now = millis();
    if (now - last_sonar_sample_ms >= kSonarSamplePeriodMs) {
        last_sonar_sample_ms = now;
        last_obstacle_detected = sonarObstacleDetectedNow();
    }
    return last_obstacle_detected;
}
