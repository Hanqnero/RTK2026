#ifndef ARDUINO_PCA9685_SERVO_OUTPUT_H
#define ARDUINO_PCA9685_SERVO_OUTPUT_H

#include <stdint.h>

#include "mearm_servo_output.h"

// Адаптер для платы PCA9685.
//
// Простыми словами:
// - MeArm просит: "на канал 0 подай импульс 1500 us";
// - этот класс переводит 1500 us в формат PCA9685;
// - PCA9685 сама генерирует PWM-сигнал для сервопривода.
class Pca9685ServoOutput : public MeArmServoOutput {
public:
    // ЧТО ТЕБЕ ЧАЩЕ ВСЕГО НУЖНО МЕНЯТЬ:
    //
    // kDefaultAddress:
    //   I2C-адрес платы PCA9685. Обычно 0x40, если перемычки A0-A5 не запаяны.
    //
    // kDefaultFrequencyHz:
    //   Частота PWM для обычных сервоприводов. Почти всегда 50 Hz.
    //
    // kDefaultMinPulseUs / kDefaultMaxPulseUs:
    //   Безопасный диапазон импульсов. Для первого запуска лучше 1000..2000 us.
    static constexpr uint8_t kDefaultAddress = 0x40;
    static constexpr uint16_t kDefaultFrequencyHz = 50;
    static constexpr uint16_t kDefaultMinPulseUs = 1000;
    static constexpr uint16_t kDefaultCenterPulseUs = 1500;
    static constexpr uint16_t kDefaultMaxPulseUs = 2000;
    static constexpr uint8_t kChannelCount = 16;

    // Запустить PCA9685.
    //
    // address:
    //   I2C-адрес платы. Если I2C scanner показывает не 0x40, поставь сюда найденный адрес.
    //
    // frequency_hz:
    //   Частота PWM. Для сервоприводов оставь 50.
    bool begin(uint8_t address = kDefaultAddress, uint16_t frequency_hz = kDefaultFrequencyHz);

    // Поставить серво на канале channel в положение pulse_us.
    //
    // channel:
    //   Номер выхода на PCA9685: 0..15.
    //
    // pulse_us:
    //   Ширина импульса:
    //   - 1000 us: одна сторона;
    //   - 1500 us: центр;
    //   - 2000 us: другая сторона.
    bool writeMicroseconds(uint8_t channel, uint16_t pulse_us) override;

    // Отключить PWM на канале channel.
    bool detach(uint8_t channel) override;

    // Изменить безопасный диапазон импульсов.
    // Используй это после калибровки, если какой-то сустав упирается в механику.
    void setPulseLimits(uint16_t min_pulse_us, uint16_t max_pulse_us);
    bool isOnline() const;
    uint8_t address() const;
    uint16_t frequencyHz() const;

private:
    bool writeReg(uint8_t reg, uint8_t value);
    bool writeRegs(uint8_t start_reg, const uint8_t* values, uint8_t length);
    bool setPwm(uint8_t channel, uint16_t on_tick, uint16_t off_tick);
    bool setFullOff(uint8_t channel);
    uint16_t pulseToTicks(uint16_t pulse_us) const;
    uint16_t clampPulse(uint16_t pulse_us) const;

    uint8_t _address = kDefaultAddress;
    uint16_t _frequency_hz = kDefaultFrequencyHz;
    uint16_t _min_pulse_us = kDefaultMinPulseUs;
    uint16_t _max_pulse_us = kDefaultMaxPulseUs;
    bool _online = false;
};

#endif  // ARDUINO_PCA9685_SERVO_OUTPUT_H
