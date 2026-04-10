/*
  pid_autotune.ino — автоматическая калибровка PID для одного борта

  Порядок работы:
    1. Загрузи скетч на Arduino
    2. Открой Serial Plotter (115200)
    3. Скетч разгоняет мотор и наблюдает скорость
    4. После теста печатает Kp, Ki, Kd в Serial Monitor
    5. Вставь значения в config.h

  Сначала калибруй левый мотор (TUNE_LEFT = true),
  затем правый (TUNE_LEFT = false).

  Мотор должен быть подвешен или робот зафиксирован!
*/

#include <sTune.h>

// ── что калибруем ───────────────────────────────
#define TUNE_LEFT true   // true = левый, false = правый

// ── пины ────────────────────────────────────────
#define LEFT_RPWM   11
#define LEFT_LPWM   10
#define RIGHT_RPWM   8
#define RIGHT_LPWM   9

// Физически провода перекрещены: левый мотор → пины 18/19, правый → 2/3
#define LEFT_ENC_A  18
#define LEFT_ENC_B  19
#define RIGHT_ENC_A  2
#define RIGHT_ENC_B  3

// ── параметры теста ─────────────────────────────
// При pwm=60 мотор даёт ~800 tps — TARGET должен быть выше стартовой скорости.
// PWM_START подбираем так чтобы начальная скорость была ниже TARGET.
// При pwm=30 мотор даёт ~300-400 tps (близко к старту), TARGET=1500 выше этого.
const float TARGET_TPS       = 1500.0f;
const float PWM_START        = 30.0f;
const float PWM_STEP         = 180.0f;
const float TEST_TIME_SEC    = 6.0f;
const float SETTLE_TIME_SEC  = 1.0f;
const uint16_t SAMPLES       = 100;

// ── переменные ──────────────────────────────────
float measured_tps = 0.0f;
float pwm_output   = PWM_START;

volatile int32_t enc_count = 0;
int32_t prev_count = 0;
uint32_t prev_time = 0;

// активные пины для выбранного мотора
uint8_t active_rpwm = 0;
uint8_t active_lpwm = 0;
uint8_t active_enc_a = 0;
uint8_t active_enc_b = 0;

// ── sTune ───────────────────────────────────────
sTune tuner(&measured_tps, &pwm_output,
            sTune::CohenCoon_PID,
            sTune::directIP,
            sTune::printALL);

// ── ISR энкодера ────────────────────────────────
// Считаем по CHANGE на канале A.
// Направление определяем по состояниям A и B.
void enc_isr() {
    bool a = digitalRead(active_enc_a);
    bool b = digitalRead(active_enc_b);

    // Для стандартного квадратурного энкодера:
    // если A == B -> одно направление, иначе другое
    if (a == b) {
        enc_count++;
    } else {
        enc_count--;
    }
}

// ── управление мотором ──────────────────────────
void set_pwm(float pwm) {
    int p = constrain((int)pwm, -255, 255);

    if (p >= 0) {
        analogWrite(active_rpwm, p);
        analogWrite(active_lpwm, 0);
    } else {
        analogWrite(active_rpwm, 0);
        analogWrite(active_lpwm, -p);
    }
}

void stop_all() {
    analogWrite(LEFT_RPWM,  0);
    analogWrite(LEFT_LPWM,  0);
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);
}

// ── setup ───────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(LEFT_RPWM,  OUTPUT);
    pinMode(LEFT_LPWM,  OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);
    stop_all();

    if (TUNE_LEFT) {
        active_rpwm  = LEFT_RPWM;
        active_lpwm  = LEFT_LPWM;
        active_enc_a = LEFT_ENC_A;
        active_enc_b = LEFT_ENC_B;
        Serial.println("Tuning LEFT motor");
    } else {
        active_rpwm  = RIGHT_RPWM;
        active_lpwm  = RIGHT_LPWM;
        active_enc_a = RIGHT_ENC_A;
        active_enc_b = RIGHT_ENC_B;
        Serial.println("Tuning RIGHT motor");
    }

    pinMode(active_enc_a, INPUT_PULLUP);
    pinMode(active_enc_b, INPUT_PULLUP);

    // Проверка, есть ли аппаратное прерывание на выбранном пине
    int irq = digitalPinToInterrupt(active_enc_a);
    if (irq == NOT_AN_INTERRUPT) {
        Serial.println("ERROR: selected ENC_A pin has no external interrupt on this board!");
        while (true) {
            stop_all();
            delay(1000);
        }
    }

    attachInterrupt(irq, enc_isr, CHANGE);

    tuner.Configure(
        TARGET_TPS * 2.0f,  // inputSpan — должен покрывать диапазон от 0 до макс скорости
        255 - (int)PWM_START, // outputSpan — диапазон выхода от PWM_START до 255
        PWM_START,          // outputStart
        PWM_STEP,           // outputStep
        TEST_TIME_SEC,
        SETTLE_TIME_SEC,
        SAMPLES
    );

    prev_time = millis();
}

// ── loop ────────────────────────────────────────
void loop() {
    // жёсткий период 20 мс
    uint32_t now = millis();
    if ((uint32_t)(now - prev_time) < 20) return;

    uint32_t dt_ms = now - prev_time;
    prev_time = now;

    // атомарное чтение счётчика
    noInterrupts();
    int32_t cnt = enc_count;
    interrupts();

    int32_t delta = cnt - prev_count;
    prev_count = cnt;

    // тики/сек
    measured_tps = (float)delta * 1000.0f / (float)dt_ms;
    if (measured_tps < 0) measured_tps = -measured_tps;

    switch (tuner.Run()) {
        case sTune::sample:
            set_pwm(pwm_output);
            Serial.print("cnt:");
            Serial.print(cnt);
            Serial.print(" delta:");
            Serial.print(delta);
            Serial.print(" tps:");
            Serial.print(measured_tps);
            Serial.print(" pwm:");
            Serial.println(pwm_output);
            break;

        case sTune::tunings:
            stop_all();
            Serial.println();
            Serial.println("=== AUTOTUNE RESULT ===");
            Serial.print("Kp = ");
            Serial.println(tuner.GetKp(), 4);
            Serial.print("Ki = ");
            Serial.println(tuner.GetKi(), 4);
            Serial.print("Kd = ");
            Serial.println(tuner.GetKd(), 4);
            Serial.println("Insert these values into config.h");
            while (true) {
                stop_all();
                delay(1000);
            }
            break;

        case sTune::runPid:
            set_pwm(0);
            break;
    }
}