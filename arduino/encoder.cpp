#include "Arduino.h"
#include "encoder.h"

#define SPEED_PERIOD_MS 100
#define SPEED_MULT (1000 / SPEED_PERIOD_MS)

Encoder* Encoder::_instances[2] = {nullptr, nullptr};

void Encoder::_isr0() {
    if (_instances[0]) _instances[0]->handleISR();
}

void Encoder::_isr1() {
    if (_instances[1]) _instances[1]->handleISR();
}

Encoder::Encoder(int A, int B, int index) : _pin_b(B), _cnt(0), _cnt_old(0), _last_time(0), _direction(false), _speed(0) {
    if (index >= 0 && index <= 1) _instances[index] = this;
    pinMode(A, INPUT);
    pinMode(B, INPUT);
    if (index == 0)
        attachInterrupt(digitalPinToInterrupt(A), _isr0, RISING);
    else
        attachInterrupt(digitalPinToInterrupt(A), _isr1, RISING);
}

void Encoder::handleISR() {
    if (digitalRead(_pin_b)) {
        _cnt++;
        _direction = true;
    } else {
        _cnt--;
        _direction = false;
    }
    uint32_t now = millis();
    if (now - _last_time >= SPEED_PERIOD_MS) {
        _speed = (int32_t)(_cnt - _cnt_old) * SPEED_MULT;
        _cnt_old = _cnt;
        _last_time = now;
    }
}
