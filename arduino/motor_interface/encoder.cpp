#include <Arduino.h>
#include <stdint.h>

#include "encoder.h"
#include "motor_interface.h"

#define SPEED_PERIOD_MS (100)
#define SPEED_MULT (1000 / SPEED_PERIOD_MS)

Encoder::Encoder(int A, int B):
    _cnt{0},
    _cnt_old{0},
    _direction{false},
    _speed{0},
    last_time{0},
    A{A}, B{B}
{
    pinMode(A, INPUT);
    pinMode(B, INPUT);
}

bool Encoder::direction() { return _direction; }
int64_t Encoder::cnt() { return _cnt; }
int32_t Encoder::speed() { return _speed; }

void Encoder::Handler() {
    if (digitalRead(B)) {
        // Clock-wise rotation
        ++_cnt;
        _direction = true;
    } else {
        // Counter Clock-wise rotation
        --_cnt;
        _direction = false;
    }

    // update speed
    if (millis() - last_time >= SPEED_PERIOD_MS) {
        _speed = (_cnt - _cnt_old) * SPEED_MULT;
        _cnt_old = _cnt;
        last_time = millis();
    }
}

Encoder left_enc(LEFT_ENC_CLK, LEFT_ENC_DT);
Encoder right_enc(RIGHT_ENC_CLK, RIGHT_ENC_DT);

void left_enc_isr_wrapper() {
    left_enc.Handler();
}

void right_enc_isr_wrapper() {
    right_enc.Handler();
}

void enc_start() {
    attachInterrupt(digitalPinToInterrupt(left_enc.A), left_enc_isr_wrapper, RISING);
    attachInterrupt(digitalPinToInterrupt(right_enc.A), right_enc_isr_wrapper, RISING);
}
