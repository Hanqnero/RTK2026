#include "sonar.h"

#include <Arduino.h>

#include "motor_interface.h"

namespace {

// Все ECHO отслеживаются опросом. За раз активен только один датчик:
// это и упрощает автомат, и не даёт соседнему HC-SR04 принять чужое эхо.
//
// Разрешение опроса определяется частотой loop(), а она между управляющими
// циклами измеряется тысячами итераций в секунду. Звук проходит сантиметр
// туда-обратно за 58 микросекунд, так что даже сотня микросекунд между
// опросами даёт погрешность около двух сантиметров при пороге в десятки.

enum class SonarState : uint8_t {
    Idle,           // ждём начала следующего измерения
    WaitEchoStart,  // импульс запущен, ждём фронта эха
    WaitEchoEnd,    // эхо началось, ждём спада
};

SonarState state = SonarState::Idle;

const uint8_t trigger_pins[kSonarCount] = {
    SONAR_FRONT_LEFT_TRIG_PIN,
    SONAR_FRONT_RIGHT_TRIG_PIN,
    SONAR_LEFT_RIGHT_TRIG_PIN,
    SONAR_LEFT_LEFT_TRIG_PIN,
    SONAR_RIGHT_RIGHT_TRIG_PIN,
    SONAR_RIGHT_LEFT_TRIG_PIN,
};

const uint8_t echo_pins[kSonarCount] = {
    SONAR_FRONT_LEFT_ECHO_PIN,
    SONAR_FRONT_RIGHT_ECHO_PIN,
    SONAR_LEFT_RIGHT_ECHO_PIN,
    SONAR_LEFT_LEFT_ECHO_PIN,
    SONAR_RIGHT_RIGHT_ECHO_PIN,
    SONAR_RIGHT_LEFT_ECHO_PIN,
};

uint32_t last_sample_start_ms = 0;
uint32_t echo_wait_start_us = 0;
uint32_t echo_rise_us = 0;
float last_distance_cm = -1.0f;
uint32_t accumulated_block_us = 0;
uint8_t active_sensor_index = 0;
SonarReading completed_reading = {};
bool completed_reading_ready = false;

void startPulse() {
    // Единственная блокирующая часть измерения: 12 микросекунд на запуск.
    const uint8_t trigger_pin = trigger_pins[active_sensor_index];
    digitalWrite(trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);
}

void finishReading(float distance_cm) {
    last_distance_cm = distance_cm;
    completed_reading.sensor_index = active_sensor_index;
    completed_reading.distance_cm = distance_cm;
    completed_reading_ready = true;
    active_sensor_index = static_cast<uint8_t>(
        (active_sensor_index + 1U) % kSonarCount);
    state = SonarState::Idle;
}

}  // namespace

void configureSonar() {
    for (uint8_t index = 0; index < kSonarCount; ++index) {
        pinMode(trigger_pins[index], OUTPUT);
        pinMode(echo_pins[index], INPUT);
        digitalWrite(trigger_pins[index], LOW);
    }

    last_sample_start_ms = millis();
}

void updateSonar() {
    const uint32_t entry_us = micros();

    switch (state) {
        case SonarState::Idle: {
            if (millis() - last_sample_start_ms < kSonarInterPingPeriodMs) {
                break;
            }

            last_sample_start_ms = millis();
            startPulse();
            echo_wait_start_us = micros();
            state = SonarState::WaitEchoStart;
            break;
        }

        case SonarState::WaitEchoStart: {
            if (digitalRead(echo_pins[active_sensor_index]) == HIGH) {
                echo_rise_us = micros();
                state = SonarState::WaitEchoEnd;
                break;
            }

            // Датчик не ответил вовсе: нет питания, нет контакта или он занят.
            if (micros() - echo_wait_start_us > kSonarEchoTimeoutUs) {
                finishReading(-1.0f);
            }
            break;
        }

        case SonarState::WaitEchoEnd: {
            const uint32_t elapsed_us = micros() - echo_rise_us;

            if (digitalRead(echo_pins[active_sensor_index]) == LOW) {
                // 58 микросекунд на сантиметр пути туда и обратно.
                finishReading(static_cast<float>(elapsed_us) / 58.0f);
                break;
            }

            // Эхо не закончилось: препятствия в пределах дальности нет.
            if (elapsed_us > kSonarEchoTimeoutUs) {
                finishReading(-1.0f);
            }
            break;
        }
    }

    accumulated_block_us += micros() - entry_us;
}

bool sonarTakeCompletedReading(SonarReading* reading) {
    if (!completed_reading_ready || reading == nullptr) {
        return false;
    }

    *reading = completed_reading;
    completed_reading_ready = false;
    return true;
}

float sonarLastDistanceCm() {
    return last_distance_cm;
}

uint32_t sonarTakeLastBlockUs() {
    const uint32_t value = accumulated_block_us;
    accumulated_block_us = 0;
    return value;
}
