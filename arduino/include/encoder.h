#pragma once

#include <Arduino.h>
#include <stdint.h>

#include <uEncoder.h>

class EncoderCounter {
public:
    EncoderCounter(uint8_t pin_a, uint8_t pin_b, bool reverse = false, uint8_t pin_mode = INPUT);

    void begin();

    // Продвинуть декодирование квадратуры. Вызывается из loop() на каждой
    // итерации, а не из обработчика прерывания: часть выводов энкодера
    // на этой плате не имеет аппаратного прерывания, поэтому опрос -
    // единственный вариант, работающий одинаково для всех каналов.
    void poll();

    // Отсчёты с прошлого вызова. Дельта обнуляется.
    int32_t readAndResetDelta();

    // Накопленное число отсчётов с момента запуска.
    //
    // Нужно для калибровки и маршрутных тестов: сумма дельт на хосте
    // сбивается при потере пакета, а накопленный счётчик - нет.
    //
    // 32 бит хватает: при kMaxWheelRpm и kEncoderCountsPerWheelRev из
    // motor_interface.h это около 4600 отсчётов в секунду, то есть
    // переполнение через недели непрерывного хода на максимальной скорости.
    int32_t readCount() const;

private:
    uEncoder _encoder;
    const bool _reverse;
    volatile int32_t _delta;
    volatile int32_t _count;
};
