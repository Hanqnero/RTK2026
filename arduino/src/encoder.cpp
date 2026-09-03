#include "encoder.h"

EncoderCounter::EncoderCounter(uint8_t pin_a, uint8_t pin_b, bool reverse, uint8_t pin_mode)
    : _encoder(pin_a, pin_b, pin_mode),
      _reverse(reverse),
      _delta(0),
      _count(0) {}

void EncoderCounter::begin() {
    // Step1 - щелчок на каждую четверть периода квадратуры. Вместе с
    // прерываниями на обоих каналах по CHANGE это даёт полное X4-декодирование:
    // четыре отсчёта на период, то есть максимальное разрешение.
    _encoder.setEncType(uEncoderVirt::Type::Step1);
    _encoder.setEncReverse(_reverse);
}

void EncoderCounter::poll() {
    // tick() сам читает оба физических вывода и возвращает, случился ли
    // поворот - тот же способ, что и в стендовом скетче, только внутри
    // библиотеки. Разница с прежним tickISR() только в служебном флаге
    // _isr, нужном библиотеке для смешанного (прерывание + опрос)
    // использования - здесь его нет, только чистый опрос.
    if (!_encoder.tick()) {
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

int32_t EncoderCounter::readCount() const {
    // На AVR чтение 32-битного значения - это четыре отдельных чтения байтов.
    // Прерывание энкодера, пришедшее между ними, дало бы смесь старого
    // и нового значения. Volatile от этого не спасает: он запрещает
    // оптимизацию, но не делает доступ атомарным.
    noInterrupts();
    const int32_t value = _count;
    interrupts();
    return value;
}
