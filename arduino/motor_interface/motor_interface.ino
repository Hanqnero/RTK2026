/*
  motor_interface.ino — прошивка дифференциального привода RTK2026-2

  Протокол Pi -> Arduino (11 байт):
    [0xA5][0x5A][target_linear_mps float32][target_angular_rps float32][checksum]

  Протокол Arduino -> Pi (11 байт):
    [0x5A][0xA5][left_cnt b0..b3][right_cnt b0..b3][checksum]

  PID библиотека: uPID (https://github.com/GyverLibs/uPID)
  Вход PID:  тики/сек (измеренная скорость за CONTROL_PERIOD_MS)
  Выход PID: PWM [-255, 255]
*/

#include <math.h>
#include <string.h>
#include <uPID.h>
#include "config.h"
#include "encoder.h"
#include "motor.h"

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// PID контроллеры для каждого борта
//
// D_INPUT    — дифференцирование по входу, а не по ошибке
//              устраняет derivative kick при резком изменении setpoint
// I_SATURATE — интегрирование только когда выход не насыщен
//              защита от integral windup
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
uPID pid_left (D_INPUT | I_SATURATE, CONTROL_PERIOD_MS);
uPID pid_right(D_INPUT | I_SATURATE, CONTROL_PERIOD_MS);

// команда движения робота, приходит с Pi
float target_linear_mps  = 0.0f;
float target_angular_rps = 0.0f;

// целевые скорости бортов (тики/сек), вычисляются уже на Arduino
int16_t target_left_tps  = 0;
int16_t target_right_tps = 0;

// время последней команды от Pi — для таймаута
uint32_t last_cmd_ms = 0;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Переменные управляющего цикла
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
uint32_t last_control_ms  = 0;
int32_t  prev_left_count  = 0;
int32_t  prev_right_count = 0;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Вспомогательные функции протокола
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

static uint8_t checksum(const uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

static int16_t clamp_i16_from_float(float value) {
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)lroundf(value);
}

static void update_target_wheel_tps() {
    const float half_wheel_sep = WHEEL_SEPARATION_M * 0.5f;
    const float left_mps  = target_linear_mps - target_angular_rps * half_wheel_sep;
    const float right_mps = target_linear_mps + target_angular_rps * half_wheel_sep;

    target_left_tps  = clamp_i16_from_float(left_mps * TICKS_PER_METER);
    target_right_tps = clamp_i16_from_float(right_mps * TICKS_PER_METER);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Чтение команды с Pi
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

static void read_command() {
    while (Serial.available() > 0) {
        // ищем первый байт заголовка
        if (Serial.peek() != CMD_HEADER_0) {
            Serial.read();
            continue;
        }
        // ждём полный пакет
        if (Serial.available() < CMD_PACKET_SIZE) return;

        uint8_t frame[CMD_PACKET_SIZE];
        Serial.readBytes(frame, CMD_PACKET_SIZE);

        // проверяем второй байт заголовка
        if (frame[1] != CMD_HEADER_1) continue;

        // проверяем контрольную сумму
        if (checksum(frame, CMD_PACKET_SIZE - 1) != frame[CMD_PACKET_SIZE - 1]) continue;

        // распаковываем float32 LE
        memcpy(&target_linear_mps, frame + 2, sizeof(float));
        memcpy(&target_angular_rps, frame + 6, sizeof(float));

        if (!isfinite(target_linear_mps) || !isfinite(target_angular_rps)) {
            continue;
        }

        update_target_wheel_tps();
        last_cmd_ms = millis();
    }
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Отправка телеметрии на Pi
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

static void send_telemetry(int32_t left_cnt, int32_t right_cnt) {
    uint8_t frame[TEL_PACKET_SIZE];
    frame[0] = TEL_HEADER_0;
    frame[1] = TEL_HEADER_1;

    // left_count (int32 LE)
    frame[2] = (left_cnt >>  0) & 0xFF;
    frame[3] = (left_cnt >>  8) & 0xFF;
    frame[4] = (left_cnt >> 16) & 0xFF;
    frame[5] = (left_cnt >> 24) & 0xFF;

    // right_count (int32 LE)
    frame[6] = (right_cnt >>  0) & 0xFF;
    frame[7] = (right_cnt >>  8) & 0xFF;
    frame[8] = (right_cnt >> 16) & 0xFF;
    frame[9] = (right_cnt >> 24) & 0xFF;

    frame[10] = checksum(frame, TEL_PACKET_SIZE - 1);
    Serial.write(frame, TEL_PACKET_SIZE);
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Setup
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.setTimeout(10);

    motors_begin();
    encoders_begin();

    // настройка PID левого борта
    pid_left.setKp(PID_KP);
    pid_left.setKi(PID_KI);
    pid_left.setKd(PID_KD);
    pid_left.outMax = PWM_MAX;
    pid_left.outMin = PWM_MIN;

    // настройка PID правого борта
    pid_right.setKp(PID_KP);
    pid_right.setKi(PID_KI);
    pid_right.setKd(PID_KD);
    pid_right.outMax = PWM_MAX;
    pid_right.outMin = PWM_MIN;

    last_cmd_ms      = millis();
    last_control_ms  = millis();
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Loop
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void loop() {
    // читаем команды от Pi на каждой итерации loop
    read_command();

    uint32_t now = millis();

    // жёсткий период управляющего цикла
    if ((uint32_t)(now - last_control_ms) < CONTROL_PERIOD_MS) return;
    last_control_ms = now;

    // таймаут команды — Pi молчит слишком долго
    if ((uint32_t)(now - last_cmd_ms) > COMMAND_TIMEOUT_MS) {
        target_linear_mps  = 0.0f;
        target_angular_rps = 0.0f;
        target_left_tps  = 0;
        target_right_tps = 0;
    }

    // читаем энкодеры атомарно
    int32_t left_cnt, right_cnt;
    noInterrupts();
    left_cnt  = enc_left.count;
    right_cnt = enc_right.count;
    interrupts();

    // применяем знаки энкодеров
    left_cnt  *= LEFT_ENC_SIGN;
    right_cnt *= RIGHT_ENC_SIGN;

    // вычисляем скорость за период (тики/сек)
    int32_t left_speed  = (int32_t)((left_cnt  - prev_left_count)  * (1000 / CONTROL_PERIOD_MS));
    int32_t right_speed = (int32_t)((right_cnt - prev_right_count) * (1000 / CONTROL_PERIOD_MS));
    prev_left_count  = left_cnt;
    prev_right_count = right_cnt;

    if (target_left_tps == 0 && target_right_tps == 0) {
        // явная остановка — не вычисляем PID, сразу стоп
        // это предотвращает integral windup при стоянке
        motor_left_set(0);
        motor_right_set(0);
        pid_left.integral  = 0;
        pid_right.integral = 0;
    } else {
        // вычисляем PID для каждого борта
        pid_left.setpoint  = (float)target_left_tps;
        pid_right.setpoint = (float)target_right_tps;

        int pwm_left  = (int)pid_left.compute(left_speed);
        int pwm_right = (int)pid_right.compute(right_speed);

        motor_left_set(pwm_left);
        motor_right_set(pwm_right);
    }

    // отправляем телеметрию (накопленные тики) на Pi
    send_telemetry(left_cnt, right_cnt);
}
