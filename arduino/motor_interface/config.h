#pragma once

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Пины моторов (BTS7960)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#define LEFT_RPWM  11
#define LEFT_LPWM  10
#define RIGHT_RPWM  8
#define RIGHT_LPWM  9

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Пины энкодеров
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Физически провода перекрещены: левый мотор → пины 18/19, правый → 2/3
#define LEFT_ENC_A  18   // канал A — прерывание (CHANGE)
#define LEFT_ENC_B  19   // канал B — направление
#define RIGHT_ENC_A  2   // канал A — прерывание (CHANGE)
#define RIGHT_ENC_B  3   // канал B — направление

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Управляющий цикл
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

// период PID цикла (мс) — uPID.setDt() должен получить то же значение
constexpr uint16_t CONTROL_PERIOD_MS  = 20;   // 50 Hz

// если Pi молчит дольше — останавливаем моторы
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// PID коэффициенты (тюнятся экспериментально)
// вход PID:  тики/сек (измеренная скорость за период)
// выход PID: PWM [-255, 255]
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
constexpr float PID_KP   = 0.8f;
constexpr float PID_KI   = 1.2f;
constexpr float PID_KD   = 0.05f;
constexpr float PWM_MAX  =  255.0f;
constexpr float PWM_MIN  = -255.0f;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Знаки моторов и энкодеров
// Меняй если мотор или энкодер физически перевёрнут
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
constexpr int LEFT_MOTOR_SIGN  =  1;
constexpr int RIGHT_MOTOR_SIGN =  1;
constexpr int LEFT_ENC_SIGN    = -1;  // канал B левого энкодера физически инвертирован
constexpr int RIGHT_ENC_SIGN   =  1;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Серийный протокол
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
constexpr uint32_t SERIAL_BAUD     = 115200;
constexpr uint8_t  CMD_HEADER_0    = 0xA5;
constexpr uint8_t  CMD_HEADER_1    = 0x5A;
constexpr uint8_t  TEL_HEADER_0    = 0x5A;
constexpr uint8_t  TEL_HEADER_1    = 0xA5;
constexpr uint8_t  CMD_PACKET_SIZE = 7;   // header(2) + tps(4) + checksum(1)
constexpr uint8_t  TEL_PACKET_SIZE = 11;  // header(2) + counts(8) + checksum(1)
