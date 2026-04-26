#include "bmi270_spi.h"

#include <SPI.h>

namespace {

constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kRegAccDataXlsb = 0x0C;
constexpr uint8_t kRegGyroDataXlsb = 0x12;
constexpr uint8_t kRegAccConf = 0x40;
constexpr uint8_t kRegAccRange = 0x41;
constexpr uint8_t kRegGyroConf = 0x42;
constexpr uint8_t kRegGyroRange = 0x43;
constexpr uint8_t kRegPwrConf = 0x7C;
constexpr uint8_t kRegPwrCtrl = 0x7D;
constexpr uint8_t kRegCmd = 0x7E;

constexpr uint8_t kChipIdBmi270 = 0x24;
constexpr uint8_t kCmdSoftReset = 0xB6;

}  // namespace

bool Bmi270Spi::begin(uint8_t cs_pin, uint32_t spi_clock_hz) {
    _cs_pin = cs_pin;
    _spi_clock_hz = spi_clock_hz;
    _spi_mode = SPI_MODE0;
    _chip_id = 0;
    _online = false;

    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    SPI.begin();
    delay(10);

    if (!probeChipId()) {
        return false;
    }

    writeReg(kRegCmd, kCmdSoftReset);
    delay(12);

    if (!probeChipId()) {
        return false;
    }

    // Disable advanced power save and enable accel/gyro.
    if (!writeReg(kRegPwrConf, 0x00)) {
        _online = false;
        return false;
    }
    delay(2);

    if (!writeReg(kRegPwrCtrl, 0x0E)) {
        _online = false;
        return false;
    }

    // Conservative defaults: 100 Hz ODR, +/-4g accel, +/-500 dps gyro.
    writeReg(kRegAccConf, 0xA8);
    writeReg(kRegAccRange, 0x01);
    writeReg(kRegGyroConf, 0xA9);
    writeReg(kRegGyroRange, 0x02);

    delay(50);
    _online = true;
    return true;
}

bool Bmi270Spi::readSample(Sample& sample) {
    if (!_online) {
        return false;
    }

    uint8_t raw[12] = {0};
    if (!readRegs(kRegAccDataXlsb, raw, sizeof(raw))) {
        return false;
    }

    sample.acc_x = static_cast<int16_t>((static_cast<uint16_t>(raw[1]) << 8) | raw[0]);
    sample.acc_y = static_cast<int16_t>((static_cast<uint16_t>(raw[3]) << 8) | raw[2]);
    sample.acc_z = static_cast<int16_t>((static_cast<uint16_t>(raw[5]) << 8) | raw[4]);
    sample.gyro_x = static_cast<int16_t>((static_cast<uint16_t>(raw[7]) << 8) | raw[6]);
    sample.gyro_y = static_cast<int16_t>((static_cast<uint16_t>(raw[9]) << 8) | raw[8]);
    sample.gyro_z = static_cast<int16_t>((static_cast<uint16_t>(raw[11]) << 8) | raw[10]);

    return true;
}

bool Bmi270Spi::isOnline() const {
    return _online;
}

uint8_t Bmi270Spi::chipId() const {
    return _chip_id;
}

bool Bmi270Spi::probeChipId() {
    static const uint8_t kModes[] = {SPI_MODE0, SPI_MODE3};

    // Probe at requested clock first, then conservative fallbacks.
    uint32_t clocks[3] = {_spi_clock_hz, 500000UL, 100000UL};
    if (_spi_clock_hz <= 500000UL) {
        clocks[1] = 250000UL;
    }
    if (_spi_clock_hz <= 100000UL) {
        clocks[2] = _spi_clock_hz;
    }

    for (uint8_t mode : kModes) {
        _spi_mode = mode;
        for (uint8_t ci = 0; ci < 3; ++ci) {
            _spi_clock_hz = clocks[ci];

            // First read after interface switch can be stale on some modules.
            uint8_t dummy = 0;
            (void)readRegs(kRegChipId, &dummy, 1);
            delay(1);

            for (uint8_t attempt = 0; attempt < 3; ++attempt) {
                _chip_id = readReg(kRegChipId);
                if (_chip_id == kChipIdBmi270) {
                    return true;
                }
                delay(1);
            }
        }
    }

    _online = false;
    return false;
}

bool Bmi270Spi::writeReg(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(SPISettings(_spi_clock_hz, MSBFIRST, _spi_mode));
    digitalWrite(_cs_pin, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(value);
    digitalWrite(_cs_pin, HIGH);
    SPI.endTransaction();
    return true;
}

uint8_t Bmi270Spi::readReg(uint8_t reg) {
    uint8_t value = 0;
    readRegs(reg, &value, 1);
    return value;
}

bool Bmi270Spi::readRegs(uint8_t start_reg, uint8_t* buffer, uint8_t length) {
    if (!buffer || !length) {
        return false;
    }

    SPI.beginTransaction(SPISettings(_spi_clock_hz, MSBFIRST, _spi_mode));
    digitalWrite(_cs_pin, LOW);
    SPI.transfer(start_reg | 0x80);
    SPI.transfer(0x00);
    for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(_cs_pin, HIGH);
    SPI.endTransaction();
    return true;
}
