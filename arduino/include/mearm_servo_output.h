#ifndef ARDUINO_MEARM_SERVO_OUTPUT_H
#define ARDUINO_MEARM_SERVO_OUTPUT_H

#include <stdint.h>

// Это общий "разъем" между библиотекой MeArm и любым способом управления сервами.
//
// MeArm не должен знать, куда реально подключены сервоприводы:
// - напрямую в PWM-пины Arduino;
// - в PCA9685 по I2C;
// - в другой драйвер.
//
// Поэтому MeArm говорит только две вещи:
// - channel: номер канала сервопривода;
// - pulse_us: ширина импульса в микросекундах.
class MeArmServoOutput {
public:
    virtual ~MeArmServoOutput() {}

    // Отправить сервоприводу команду положения.
    //
    // Пример:
    // - channel = 0: первый выход PCA9685;
    // - pulse_us = 1500: поставить серво примерно в центр.
    virtual bool writeMicroseconds(uint8_t channel, uint16_t pulse_us) = 0;

    // Отключить PWM на канале. Серво перестанет получать управляющие импульсы.
    virtual bool detach(uint8_t channel) = 0;
};

#endif  // ARDUINO_MEARM_SERVO_OUTPUT_H
