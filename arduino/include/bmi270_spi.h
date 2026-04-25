#pragma once

#include <Arduino.h>
#include <stdint.h>

class Bmi270Spi {
public:
    struct Sample {
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    };

    bool begin(uint8_t cs_pin, uint32_t spi_clock_hz);
    bool readSample(Sample& sample);

    bool isOnline() const;
    uint8_t chipId() const;

private:
    bool writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    bool readRegs(uint8_t start_reg, uint8_t* buffer, uint8_t length);

    uint8_t _cs_pin = 0;
    uint32_t _spi_clock_hz = 1000000UL;
    uint8_t _chip_id = 0;
    bool _online = false;
};
