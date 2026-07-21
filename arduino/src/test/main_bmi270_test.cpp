#include <Arduino.h>

#include "bmi270_i2c.h"

namespace {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kImuI2cClockHz = 400000UL;
constexpr uint32_t kSamplePeriodMs = 100;
constexpr uint32_t kRetryPeriodMs = 1000;
constexpr uint8_t kCandidateAddresses[] = {0x68, 0x69};

Bmi270I2c imu;
uint32_t last_sample_ms = 0;
uint32_t last_retry_ms = 0;
bool imu_online = false;

void printHexByte(uint8_t value) {
    if (value < 0x10) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

void printAddress(uint8_t address) {
    Serial.print(F("0x"));
    printHexByte(address);
}

void printImuStatus(const __FlashStringHelper* label, bool ok) {
    Serial.print(label);
    Serial.print(F(" begin="));
    Serial.print(ok ? F("ok") : F("fail"));
    Serial.print(F(" addr="));
    printAddress(imu.address());
    Serial.print(F(" chip_id=0x"));
    printHexByte(imu.chipId());
    Serial.print(F(" internal_status=0x"));
    printHexByte(imu.internalStatus());
    Serial.println();
}

bool beginAnyAddress(const __FlashStringHelper* label) {
    for (uint8_t i = 0; i < sizeof(kCandidateAddresses); ++i) {
        const bool ok = imu.begin(kCandidateAddresses[i], kImuI2cClockHz);
        printImuStatus(label, ok);
        if (ok) {
            return true;
        }
    }
    return false;
}

void printSample(uint32_t now, const Bmi270I2c::Sample& sample) {
    Serial.print(F("t_ms="));
    Serial.print(now);
    Serial.print(F(" addr="));
    printAddress(imu.address());
    Serial.print(F(" acc=("));
    Serial.print(sample.acc_x);
    Serial.print(F(", "));
    Serial.print(sample.acc_y);
    Serial.print(F(", "));
    Serial.print(sample.acc_z);
    Serial.print(F(") gyro=("));
    Serial.print(sample.gyro_x);
    Serial.print(F(", "));
    Serial.print(sample.gyro_y);
    Serial.print(F(", "));
    Serial.print(sample.gyro_z);
    Serial.println(F(")"));
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaud);
    Serial.setTimeout(5);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println(F("BMI270 I2C polling test"));
    Serial.println(F("Mega SDA=D20 SCL=D21 I2C=400000Hz sample_period=100ms"));

    imu_online = beginAnyAddress(F("startup"));
    digitalWrite(LED_BUILTIN, imu_online ? HIGH : LOW);
}

void loop() {
    const uint32_t now = millis();
    if (!imu_online) {
        if ((uint32_t)(now - last_retry_ms) >= kRetryPeriodMs) {
            last_retry_ms = now;
            imu_online = beginAnyAddress(F("retry"));
            digitalWrite(LED_BUILTIN, imu_online ? HIGH : LOW);
        }
        return;
    }

    if ((uint32_t)(now - last_sample_ms) < kSamplePeriodMs) {
        return;
    }
    last_sample_ms = now;

    Bmi270I2c::Sample sample = {};
    if (imu.readSample(sample)) {
        printSample(now, sample);
    } else {
        Serial.println(F("sample_fail: IMU read failed, retrying init"));
        imu_online = false;
        digitalWrite(LED_BUILTIN, LOW);
    }
}
