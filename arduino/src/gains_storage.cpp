#include "gains_storage.h"

#include <EEPROM.h>
#include <math.h>
#include <string.h>

#include "frame.h"

namespace {

constexpr uint32_t kGainsMagic = 0x524B3257UL;
constexpr uint8_t kGainsFormatVersion = 1;
constexpr int kGainsEepromAddress = 0;

struct __attribute__((packed)) GainsRecord {
    uint32_t magic;
    uint8_t version;
    WheelGains left;
    WheelGains right;
    uint16_t crc;
};

// Размер записи фиксируется: изменение WheelGains обязано ломать сборку,
// а не молча читать чужие байты как коэффициенты.
static_assert(sizeof(WheelGains) == 20, "WheelGains изменился");
static_assert(sizeof(GainsRecord) == 47, "GainsRecord изменился");

uint16_t recordChecksum(const GainsRecord& record) {
    // CRC считается по всем байтам записи, кроме самого поля crc.
    return crc16Ccitt(
        reinterpret_cast<const uint8_t*>(&record),
        sizeof(GainsRecord) - sizeof(uint16_t));
}

bool gainsAreFinite(const WheelGains& gains) {
    return isfinite(gains.kp) && isfinite(gains.ki) && isfinite(gains.kd) &&
           isfinite(gains.k_static) && isfinite(gains.k_velocity);
}

void readRecord(GainsRecord* record) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(record);

    for (uint16_t index = 0; index < sizeof(GainsRecord); ++index) {
        bytes[index] = EEPROM.read(kGainsEepromAddress + index);
    }
}

void writeRecord(const GainsRecord& record) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&record);

    for (uint16_t index = 0; index < sizeof(GainsRecord); ++index) {
        // update, а не write: неизменившиеся байты не переписываются
        // и не расходуют ресурс ячейки.
        EEPROM.update(kGainsEepromAddress + index, bytes[index]);
    }
}

}  // namespace

GainsLoadResult loadGains(StoredGains* out) {
    if (out == nullptr) {
        return GainsLoadResult::InvalidValues;
    }

    GainsRecord record;
    readRecord(&record);

    if (record.magic != kGainsMagic) {
        return GainsLoadResult::NotInitialised;
    }

    if (record.version != kGainsFormatVersion) {
        return GainsLoadResult::VersionMismatch;
    }

    if (recordChecksum(record) != record.crc) {
        return GainsLoadResult::CorruptedChecksum;
    }

    if (!gainsAreFinite(record.left) || !gainsAreFinite(record.right)) {
        return GainsLoadResult::InvalidValues;
    }

    out->left = record.left;
    out->right = record.right;

    return GainsLoadResult::Ok;
}

bool saveGains(const StoredGains& gains) {
    if (!gainsAreFinite(gains.left) || !gainsAreFinite(gains.right)) {
        return false;
    }

    GainsRecord record;
    memset(&record, 0, sizeof(record));

    record.magic = kGainsMagic;
    record.version = kGainsFormatVersion;
    record.left = gains.left;
    record.right = gains.right;
    record.crc = recordChecksum(record);

    writeRecord(record);

    // Запись подтверждается обратным чтением: молчаливый отказ EEPROM
    // выглядел бы для оператора как удачно сохранённая настройка.
    StoredGains verification;
    return loadGains(&verification) == GainsLoadResult::Ok &&
           memcmp(&verification, &gains, sizeof(StoredGains)) == 0;
}

void clearStoredGains() {
    // Достаточно испортить magic: остальные байты станут неинтерпретируемыми.
    for (uint16_t index = 0; index < sizeof(uint32_t); ++index) {
        EEPROM.update(kGainsEepromAddress + index, 0xFF);
    }
}
