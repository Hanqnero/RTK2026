#pragma once

#include <Arduino.h>
#include <stdint.h>

#include <uEncoder.h>

class EncoderCounter {
public:
    EncoderCounter(uint8_t pin_a, uint8_t pin_b, bool reverse = false, uint8_t pin_mode = INPUT);

    void begin();
    void handleInterrupt();

    int32_t readAndResetDelta();
    int64_t readCount() const;

private:
    static int8_t directionFromState(uEncoderVirt::State state);

    uEncoder _encoder;
    const bool _reverse;
    volatile int32_t _delta;
    volatile int64_t _count;
};
