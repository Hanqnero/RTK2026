#pragma once

#include <Arduino.h>
#include <stdint.h>

class Bmi270I2c {
public:
    struct Sample {
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    };

    bool begin(uint8_t i2c_address, uint32_t i2c_clock_hz);
    bool readSample(Sample& sample);

    bool isOnline() const;
    uint8_t address() const;
    uint8_t chipId() const;
    uint8_t internalStatus() const;

private:
    bool probeChipId();
    bool loadConfig();
    bool configureSensors();
    bool writeRegChecked(uint8_t reg, uint8_t value);
    bool writeReg(uint8_t reg, uint8_t value);
    bool writeRegs(uint8_t start_reg, const uint8_t* buffer, uint8_t length);
    bool writeProgmemRegs(uint8_t start_reg, const uint8_t* progmem_buffer, uint16_t offset, uint8_t length);
    uint8_t readReg(uint8_t reg);
    bool readRegs(uint8_t start_reg, uint8_t* buffer, uint8_t length);

    uint8_t _address = 0x68;
    uint32_t _i2c_clock_hz = 400000UL;
    uint8_t _chip_id = 0;
    uint8_t _internal_status = 0;
    bool _online = false;
};
