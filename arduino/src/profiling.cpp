#include "profiling.h"

#include <Arduino.h>

extern "C" char* __brkval;
extern "C" char __heap_start;

uint16_t freeRamBytes() {
    char stack_top;

    // Если куча ещё ни разу не расширялась, __brkval равен нулю,
    // и границей служит начало кучи из линкер-скрипта.
    const char* heap_end = (__brkval == nullptr) ? &__heap_start : __brkval;
    const int32_t free_bytes = &stack_top - heap_end;

    if (free_bytes < 0) {
        return 0;
    }

    if (free_bytes > 0xFFFF) {
        return 0xFFFFU;
    }

    return static_cast<uint16_t>(free_bytes);
}

LoopProfiler::LoopProfiler() {
    resetAll();
}

void LoopProfiler::recordCycle(uint32_t dt_us, uint32_t duration_us) {
    ++_cycle_count;

    if (dt_us < _window_min_dt_us) {
        _window_min_dt_us = dt_us;
    }

    if (dt_us > _window_max_dt_us) {
        _window_max_dt_us = dt_us;
    }

    _window_sum_dt_us += dt_us;
    ++_window_samples;

    if (duration_us > _window_max_duration_us) {
        _window_max_duration_us = duration_us;
    }
}

void LoopProfiler::recordSonarBlock(uint32_t duration_us) {
    if (duration_us == 0) {
        return;
    }

    if (duration_us > _window_max_sonar_us) {
        _window_max_sonar_us = duration_us;
    }
}

void LoopProfiler::recordOverrun() {
    ++_overrun_count;
}

uint32_t LoopProfiler::windowMinDtUs() const {
    // До первой выборки минимум равен «бесконечности» и наружу его отдавать
    // нельзя: хост принял бы 0xFFFFFFFF за реальное измерение.
    return (_window_samples == 0) ? 0U : _window_min_dt_us;
}

uint32_t LoopProfiler::windowMeanDtUs() const {
    if (_window_samples == 0) {
        return 0U;
    }

    return _window_sum_dt_us / _window_samples;
}

void LoopProfiler::resetWindow() {
    _window_min_dt_us = 0xFFFFFFFFUL;
    _window_max_dt_us = 0;
    _window_sum_dt_us = 0;
    _window_samples = 0;
    _window_max_duration_us = 0;
    _window_max_sonar_us = 0;
}

void LoopProfiler::resetAll() {
    _cycle_count = 0;
    _overrun_count = 0;
    resetWindow();
}
