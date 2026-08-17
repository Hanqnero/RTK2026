#pragma once

#include <stdint.h>

// Свободная RAM между вершиной кучи и текущей вершиной стека.
// На ATmega2560 всего 8 КБ, поэтому падение этого числа со временем —
// первый признак утечки или слишком глубокой рекурсии в прерываниях.
uint16_t freeRamBytes();

// Накопитель статистики управляющего цикла.
//
// Разделение намеренное: min/max/mean считаются в скользящем окне между
// отправками StatsPayload, а счётчики событий накапливаются от старта.
// Окно отвечает на вопрос «как система ведёт себя сейчас», счётчики —
// на вопрос «сколько всего было сбоев».
class LoopProfiler {
public:
    LoopProfiler();

    // dt_us — фактический интервал от предыдущего цикла.
    // duration_us — сколько занял сам расчёт цикла.
    void recordCycle(uint32_t dt_us, uint32_t duration_us);

    // Длительность блокирующего измерения сонара. Ноль означает, что на этом
    // цикле измерения не было, и такой вызов игнорируется.
    void recordSonarBlock(uint32_t duration_us);

    void recordOverrun();

    uint32_t cycleCount() const { return _cycle_count; }
    uint16_t overrunCount() const { return _overrun_count; }

    uint32_t windowMinDtUs() const;
    uint32_t windowMaxDtUs() const { return _window_max_dt_us; }
    uint32_t windowMeanDtUs() const;
    uint32_t windowMaxCycleDurationUs() const { return _window_max_duration_us; }
    uint32_t windowMaxSonarBlockUs() const { return _window_max_sonar_us; }

    // Начать новое окно. Накопительные счётчики не трогаются.
    void resetWindow();

    // Полный сброс, включая накопительные счётчики.
    void resetAll();

private:
    uint32_t _cycle_count;
    uint16_t _overrun_count;

    uint32_t _window_min_dt_us;
    uint32_t _window_max_dt_us;
    uint32_t _window_sum_dt_us;
    uint32_t _window_samples;
    uint32_t _window_max_duration_us;
    uint32_t _window_max_sonar_us;
};
