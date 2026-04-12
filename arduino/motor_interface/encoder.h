#pragma once
#include <Arduino.h>
#include "config.h"

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Энкодер в режиме 2x квадратуры (CHANGE на канале A)
//
// Направление определяется состоянием каналов A и B
// в момент прерывания:
//   A == B → вперёд (+1)
//   A != B → назад  (-1)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

struct Encoder {
    const uint8_t pin_a;
    const uint8_t pin_b;

    volatile int32_t count = 0;  // накопленные тики с момента старта

    Encoder(uint8_t a, uint8_t b) : pin_a(a), pin_b(b) {}

    void begin() {
        pinMode(pin_a, INPUT_PULLUP);
        pinMode(pin_b, INPUT_PULLUP);
    }

    // вызывается из ISR на CHANGE канала A
    void on_change() {
        bool a = digitalRead(pin_a);
        bool b = digitalRead(pin_b);
        if (a == b) ++count;
        else        --count;
    }
};

extern Encoder enc_left;
extern Encoder enc_right;

// ISR обёртки — attachInterrupt не принимает методы класса
void enc_left_isr();
void enc_right_isr();

void encoders_begin();
