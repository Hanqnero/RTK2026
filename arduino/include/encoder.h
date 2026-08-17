#pragma once

#include <Arduino.h>
#include <stdint.h>

#include <uEncoder.h>

class EncoderCounter {
public:
    EncoderCounter(uint8_t pin_a, uint8_t pin_b, bool reverse = false, uint8_t pin_mode = INPUT);

    void begin();
    void handleInterrupt();

    // Отсчёты с прошлого вызова. Дельта обнуляется.
    int32_t readAndResetDelta();

    // Накопленное число отсчётов с момента запуска.
    //
    // Нужно для калибровки и маршрутных тестов: сумма дельт на хосте
    // сбивается при потере пакета, а накопленный счётчик - нет.
    //
    // 32 бит хватает: при 960 об/мин и ~229 отсчётах на оборот колеса это
    // около 3700 отсчётов в секунду, то есть переполнение через недели
    // непрерывного хода на максимальной скорости.
    int32_t readCount() const;

private:
    uEncoder _encoder;
    const bool _reverse;
    volatile int32_t _delta;
    volatile int32_t _count;
};
