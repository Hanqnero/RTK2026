#include "bmi270_i2c.h"

#include <Wire.h>

#include "bmi270_config.h"

namespace {

constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kRegAccDataXlsb = 0x0C;
constexpr uint8_t kRegInternalStatus = 0x21;
constexpr uint8_t kRegAccConf = 0x40;
constexpr uint8_t kRegAccRange = 0x41;
constexpr uint8_t kRegGyroConf = 0x42;
constexpr uint8_t kRegGyroRange = 0x43;
constexpr uint8_t kRegInitCtrl = 0x59;
constexpr uint8_t kRegInitAddr0 = 0x5B;
constexpr uint8_t kRegInitData = 0x5E;
constexpr uint8_t kRegPwrConf = 0x7C;
constexpr uint8_t kRegPwrCtrl = 0x7D;
constexpr uint8_t kRegCmd = 0x7E;

constexpr uint8_t kChipIdBmi270 = 0x24;
constexpr uint8_t kCmdSoftReset = 0xB6;
constexpr uint8_t kConfigLoadSuccess = 0x01;
constexpr uint8_t kConfigLoadStatusMask = 0x0F;
constexpr uint8_t kConfigChunkSize = 16;

}  // namespace

bool Bmi270I2c::begin(uint8_t i2c_address, uint32_t i2c_clock_hz) {
    _address = i2c_address;
    _i2c_clock_hz = i2c_clock_hz;
    _chip_id = 0;
    _internal_status = 0;
    _online = false;

    Wire.begin();
    Wire.setClock(_i2c_clock_hz);
    Wire.setWireTimeout(25000, true);
    delay(10);

    if (!probeChipId()) {
        return false;
    }

    writeReg(kRegCmd, kCmdSoftReset);
    delay(12);

    if (!probeChipId()) {
        return false;
    }

    if (!loadConfig()) {
        return false;
    }

    if (!configureSensors()) {
        return false;
    }

    _online = true;
    return true;
}

bool Bmi270I2c::readSample(Sample& sample) {
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

bool Bmi270I2c::isOnline() const {
    return _online;
}

uint8_t Bmi270I2c::address() const {
    return _address;
}

uint8_t Bmi270I2c::chipId() const {
    return _chip_id;
}

uint8_t Bmi270I2c::internalStatus() const {
    return _internal_status;
}

bool Bmi270I2c::probeChipId() {
    for (uint8_t attempt = 0; attempt < 5; ++attempt) {
        _chip_id = readReg(kRegChipId);
        if (_chip_id == kChipIdBmi270) {
            return true;
        }
        delay(2);
    }

    _online = false;
    return false;
}

bool Bmi270I2c::loadConfig() {
    if (!writeReg(kRegPwrConf, 0x00)) {
        _online = false;
        return false;
    }
    delayMicroseconds(500);

    if (!writeReg(kRegInitCtrl, 0x00)) {
        _online = false;
        return false;
    }
    delayMicroseconds(500);

    for (uint16_t index = 0; index < kBmi270ConfigSize; index += kConfigChunkSize) {
        const uint16_t remaining = kBmi270ConfigSize - index;
        const uint8_t chunk_length = static_cast<uint8_t>(
            remaining < kConfigChunkSize ? remaining : kConfigChunkSize);
        const uint16_t word_address = index / 2;
        const uint8_t addr[2] = {
            static_cast<uint8_t>(word_address & 0x0F),
            static_cast<uint8_t>(word_address >> 4),
        };

        if (!writeRegs(kRegInitAddr0, addr, sizeof(addr)) ||
            !writeProgmemRegs(kRegInitData, kBmi270ConfigFile, index, chunk_length)) {
            _online = false;
            return false;
        }
    }

    if (!writeReg(kRegInitCtrl, 0x01)) {
        _online = false;
        return false;
    }

    delay(20);
    _internal_status = readReg(kRegInternalStatus);
    if ((_internal_status & kConfigLoadStatusMask) != kConfigLoadSuccess) {
        _online = false;
        return false;
    }

    return true;
}

bool Bmi270I2c::configureSensors() {
    // Значения по умолчанию режима performance: частота выдачи 100 Гц,
    // акселерометр +/-4g, гироскоп +/-500 градусов в секунду.
    if (!writeRegChecked(kRegPwrConf, 0x00)) {
        _online = false;
        return false;
    }
    delay(2);

    if (!writeRegChecked(kRegAccConf, 0xA8) ||
        !writeRegChecked(kRegAccRange, 0x01) ||
        !writeRegChecked(kRegGyroConf, 0xA9) ||
        !writeRegChecked(kRegGyroRange, 0x02) ||
        !writeRegChecked(kRegPwrCtrl, 0x06)) {
        _online = false;
        return false;
    }

    delay(50);
    return true;
}

bool Bmi270I2c::writeRegChecked(uint8_t reg, uint8_t value) {
    if (!writeReg(reg, value)) {
        return false;
    }
    delayMicroseconds(2);
    return readReg(reg) == value;
}

bool Bmi270I2c::writeReg(uint8_t reg, uint8_t value) {
    return writeRegs(reg, &value, 1);
}

bool Bmi270I2c::writeRegs(uint8_t start_reg, const uint8_t* buffer, uint8_t length) {
    if (!buffer || !length) {
        return false;
    }

    Wire.beginTransmission(_address);
    Wire.write(start_reg);
    Wire.write(buffer, length);
    const bool ok = (Wire.endTransmission() == 0);
    delayMicroseconds(2);
    return ok;
}

bool Bmi270I2c::writeProgmemRegs(uint8_t start_reg, const uint8_t* progmem_buffer, uint16_t offset, uint8_t length) {
    if (!progmem_buffer || !length) {
        return false;
    }

    Wire.beginTransmission(_address);
    Wire.write(start_reg);
    for (uint8_t i = 0; i < length; ++i) {
        Wire.write(pgm_read_byte(progmem_buffer + offset + i));
    }
    const bool ok = (Wire.endTransmission() == 0);
    delayMicroseconds(2);
    return ok;
}

uint8_t Bmi270I2c::readReg(uint8_t reg) {
    uint8_t value = 0;
    readRegs(reg, &value, 1);
    return value;
}

bool Bmi270I2c::readRegs(uint8_t start_reg, uint8_t* buffer, uint8_t length) {
    if (!buffer || !length) {
        return false;
    }

    Wire.beginTransmission(_address);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(_address, length) != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; ++i) {
        buffer[i] = static_cast<uint8_t>(Wire.read());
    }

    return true;
}
