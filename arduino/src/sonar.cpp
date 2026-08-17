#include "sonar.h"

#include <Arduino.h>

#include "motor_interface.h"

namespace {

// Вывод ECHO (41) на Mega не поддерживает ни внешнее прерывание, ни pin change:
// внешние доступны только на 2, 3, 18, 19, 20, 21, а четыре из них уже заняты
// энкодерами. Поэтому эхо отслеживается опросом.
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

uint32_t last_sample_start_ms = 0;
uint32_t echo_wait_start_us = 0;
uint32_t echo_rise_us = 0;
float last_distance_cm = -1.0f;
uint32_t accumulated_block_us = 0;

void startPulse() {
    // Единственная блокирующая часть измерения: 12 микросекунд на запуск.
    digitalWrite(SONAR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG_PIN, LOW);
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

    last_sample_start_ms = millis();
}

void updateSonar() {
    const uint32_t entry_us = micros();

    switch (state) {
        case SonarState::Idle: {
            if (millis() - last_sample_start_ms < kSonarSamplePeriodMs) {
                break;
            }

            last_sample_start_ms = millis();
            startPulse();
            echo_wait_start_us = micros();
            state = SonarState::WaitEchoStart;
            break;
        }

        case SonarState::WaitEchoStart: {
            if (digitalRead(SONAR_ECHO_PIN) == HIGH) {
                echo_rise_us = micros();
                state = SonarState::WaitEchoEnd;
                break;
            }

            // Датчик не ответил вовсе: нет питания, нет контакта или он занят.
            if (micros() - echo_wait_start_us > kSonarEchoTimeoutUs) {
                last_distance_cm = -1.0f;
                state = SonarState::Idle;
            }
            break;
        }

        case SonarState::WaitEchoEnd: {
            const uint32_t elapsed_us = micros() - echo_rise_us;

            if (digitalRead(SONAR_ECHO_PIN) == LOW) {
                // 58 микросекунд на сантиметр пути туда и обратно.
                last_distance_cm = static_cast<float>(elapsed_us) / 58.0f;
                state = SonarState::Idle;
                break;
            }

            // Эхо не закончилось: препятствия в пределах дальности нет.
            if (elapsed_us > kSonarEchoTimeoutUs) {
                last_distance_cm = -1.0f;
                state = SonarState::Idle;
            }
            break;
        }
    }

    accumulated_block_us += micros() - entry_us;
}

float sonarLastDistanceCm() {
    return last_distance_cm;
}

uint32_t sonarTakeLastBlockUs() {
    const uint32_t value = accumulated_block_us;
    accumulated_block_us = 0;
    return value;
}
