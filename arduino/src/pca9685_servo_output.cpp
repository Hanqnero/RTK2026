#include "pca9685_servo_output.h"

#include <Arduino.h>
#include <Wire.h>

namespace {

// Регистры PCA9685.
// Arduino пишет туда байты по I2C, а PCA9685 меняет свое поведение.
constexpr uint8_t kRegMode1 = 0x00;
constexpr uint8_t kRegMode2 = 0x01;
constexpr uint8_t kRegLed0OnL = 0x06;
constexpr uint8_t kRegPreScale = 0xFE;

// Биты режимов PCA9685.
// Sleep нужен, чтобы безопасно поменять частоту PWM.
// AutoIncrement позволяет записывать несколько соседних регистров одной I2C-передачей.
constexpr uint8_t kMode1Restart = 0x80;
constexpr uint8_t kMode1AutoIncrement = 0x20;
constexpr uint8_t kMode1Sleep = 0x10;
constexpr uint8_t kMode2OutputDriver = 0x04;
constexpr uint8_t kFullOffBit = 0x10;

// PCA9685 обычно имеет внутренний генератор 25 MHz.
// Один PWM-период внутри чипа делится на 4096 маленьких шагов, их часто называют ticks.
constexpr uint32_t kOscillatorHz = 25000000UL;
constexpr uint16_t kPwmTicksPerPeriod = 4096;

// Начинаем с 100 kHz: это самая спокойная и совместимая скорость I2C.
// Когда все заработает стабильно, можно попробовать 400000UL.
constexpr uint32_t kI2cClockHz = 100000UL;
constexpr uint32_t kI2cTimeoutUs = 25000UL;

}  // namespace

constexpr uint8_t Pca9685ServoOutput::kDefaultAddress;
constexpr uint16_t Pca9685ServoOutput::kDefaultFrequencyHz;
constexpr uint16_t Pca9685ServoOutput::kDefaultMinPulseUs;
constexpr uint16_t Pca9685ServoOutput::kDefaultCenterPulseUs;
constexpr uint16_t Pca9685ServoOutput::kDefaultMaxPulseUs;
constexpr uint8_t Pca9685ServoOutput::kChannelCount;

bool Pca9685ServoOutput::begin(uint8_t address, uint16_t frequency_hz) {
    if (frequency_hz == 0) {
        return false;
    }

    _address = address;
    _frequency_hz = frequency_hz;
    _online = false;

    Wire.begin();
    Wire.setClock(kI2cClockHz);
    Wire.setWireTimeout(kI2cTimeoutUs, true);

    // PRE_SCALE задает частоту PWM.
    //
    // Для сервоприводов нужна частота около 50 Hz:
    // - один период = 20 ms = 20000 us;
    // - внутри периода PCA9685 считает 4096 ticks.
    //
    // Формула из datasheet PCA9685:
    // prescale = oscillator / (4096 * frequency) - 1
    const uint32_t denominator = static_cast<uint32_t>(kPwmTicksPerPeriod) * _frequency_hz;
    uint32_t prescale_value = ((kOscillatorHz + (denominator / 2UL)) / denominator) - 1UL;
    if (prescale_value > 255UL) {
        prescale_value = 255UL;
    }

    // Чтобы поменять PRE_SCALE, PCA9685 сначала переводится в sleep.
    // Потом записываем делитель частоты, режим выходов и снова запускаем драйвер.
    if (!writeReg(kRegMode1, kMode1Sleep | kMode1AutoIncrement)) {
        return false;
    }
    if (!writeReg(kRegPreScale, static_cast<uint8_t>(prescale_value))) {
        return false;
    }
    if (!writeReg(kRegMode2, kMode2OutputDriver)) {
        return false;
    }
    if (!writeReg(kRegMode1, kMode1AutoIncrement)) {
        return false;
    }

    delay(5);

    if (!writeReg(kRegMode1, kMode1Restart | kMode1AutoIncrement)) {
        return false;
    }

    _online = true;

    // После запуска ставим все 16 каналов в центр 1500 us.
    // Это безопаснее, чем оставить каналы в случайном состоянии.
    for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
        if (!writeMicroseconds(channel, kDefaultCenterPulseUs)) {
            _online = false;
            return false;
        }
    }

    return true;
}

bool Pca9685ServoOutput::writeMicroseconds(uint8_t channel, uint16_t pulse_us) {
    if (!_online || channel >= kChannelCount) {
        return false;
    }

    // Для сервопривода обычно достаточно начать импульс с tick 0,
    // а закончить на tick, который соответствует нужной ширине pulse_us.
    return setPwm(channel, 0, pulseToTicks(clampPulse(pulse_us)));
}

bool Pca9685ServoOutput::detach(uint8_t channel) {
    if (!_online || channel >= kChannelCount) {
        return false;
    }

    return setFullOff(channel);
}

void Pca9685ServoOutput::setPulseLimits(uint16_t min_pulse_us, uint16_t max_pulse_us) {
    if (min_pulse_us <= max_pulse_us) {
        _min_pulse_us = min_pulse_us;
        _max_pulse_us = max_pulse_us;
    }
}

bool Pca9685ServoOutput::isOnline() const {
    return _online;
}

uint8_t Pca9685ServoOutput::address() const {
    return _address;
}

uint16_t Pca9685ServoOutput::frequencyHz() const {
    return _frequency_hz;
}

bool Pca9685ServoOutput::writeReg(uint8_t reg, uint8_t value) {
    return writeRegs(reg, &value, 1);
}

bool Pca9685ServoOutput::writeRegs(uint8_t start_reg, const uint8_t* values, uint8_t length) {
    if (!values || length == 0) {
        return false;
    }

    Wire.beginTransmission(_address);
    Wire.write(start_reg);
    Wire.write(values, length);
    const bool ok = (Wire.endTransmission() == 0);
    delayMicroseconds(2);
    return ok;
}

bool Pca9685ServoOutput::setPwm(uint8_t channel, uint16_t on_tick, uint16_t off_tick) {
    // У каждого канала 4 регистра:
    // - когда включить HIGH;
    // - когда выключить HIGH.
    //
    // Канал 0 начинается с LED0_ON_L.
    // Канал 1 находится на 4 байта дальше, канал 2 еще на 4 байта дальше.
    const uint8_t reg = static_cast<uint8_t>(kRegLed0OnL + (4U * channel));
    const uint8_t values[4] = {
        static_cast<uint8_t>(on_tick & 0xFF),
        static_cast<uint8_t>((on_tick >> 8) & 0x0F),
        static_cast<uint8_t>(off_tick & 0xFF),
        static_cast<uint8_t>((off_tick >> 8) & 0x0F),
    };
    return writeRegs(reg, values, sizeof(values));
}

bool Pca9685ServoOutput::setFullOff(uint8_t channel) {
    // Full-off bit говорит PCA9685 полностью выключить PWM на этом канале.
    const uint8_t reg = static_cast<uint8_t>(kRegLed0OnL + (4U * channel));
    const uint8_t values[4] = {0, 0, 0, kFullOffBit};
    return writeRegs(reg, values, sizeof(values));
}

uint16_t Pca9685ServoOutput::pulseToTicks(uint16_t pulse_us) const {
    // Перевод микросекунд в ticks PCA9685.
    //
    // При 50 Hz:
    // - период = 20000 us;
    // - 4096 ticks на период;
    // - 1500 us примерно равно 307 ticks.
    const uint32_t period_us = 1000000UL / _frequency_hz;
    uint32_t ticks = ((static_cast<uint32_t>(pulse_us) * kPwmTicksPerPeriod) + (period_us / 2UL)) / period_us;
    if (ticks >= kPwmTicksPerPeriod) {
        ticks = kPwmTicksPerPeriod - 1U;
    }
    return static_cast<uint16_t>(ticks);
}

uint16_t Pca9685ServoOutput::clampPulse(uint16_t pulse_us) const {
    if (pulse_us < _min_pulse_us) {
        return _min_pulse_us;
    }
    if (pulse_us > _max_pulse_us) {
        return _max_pulse_us;
    }
    return pulse_us;
}
