#include "encoder.h"

EncoderCounter::EncoderCounter(uint8_t pin_a, uint8_t pin_b, bool reverse, uint8_t pin_mode)
    : _encoder(pin_a, pin_b, pin_mode),
      _reverse(reverse),
      _delta(0),
      _count(0) {}

void EncoderCounter::begin() {
    _encoder.setEncType(uEncoderVirt::Type::Step1);
    _encoder.setEncReverse(_reverse);
}

void EncoderCounter::handleInterrupt() {
    _encoder.tickISR();
    if (!_encoder.turn()) {
        return;
    }

    const int8_t dir = _encoder.dir();
    _delta += dir;
    _count += dir;
}

int32_t EncoderCounter::readAndResetDelta() {
    noInterrupts();
    const int32_t delta = _delta;
    _delta = 0;
    interrupts();
    return delta;
}

int64_t EncoderCounter::readCount() const {
    // Use volatile read to safely access shared counter
    return _count;
}
