#pragma once

#include <stdint.h>

#include "control_protocol.h"

// CRC16-CCITT, полином 0x1021, начальное значение 0xFFFF.
// Табличная реализация на AVR стоила бы 512 байт flash, поэтому здесь
// побитовый вариант: при 55-байтовых кадрах и 50 Гц это единицы процентов
// одного управляющего цикла.
uint16_t crc16Ccitt(const uint8_t* data, uint8_t length, uint16_t seed = 0xFFFFU);

// Собрать готовый кадр в out.
// Возвращает полную длину кадра либо 0, если буфер слишком мал.
uint8_t buildFrame(
    uint8_t* out,
    uint8_t out_capacity,
    uint8_t message_id,
    const void* payload,
    uint8_t payload_length);

// Побайтовый разборщик входящего потока.
//
// Разборщик не владеет serial-портом и не выделяет память: вызывающий код
// скармливает ему байты по одному и после каждого проверяет возвращённое
// значение. Это позволяет обрабатывать поток без промежуточного буфера
// на стороне main.cpp.
class FrameParser {
public:
    FrameParser();

    // Обработать один байт. Возвращает true, если только что собран
    // корректный кадр: messageId(), payload() и payloadLength() при этом
    // действительны до следующего вызова feed().
    bool feed(uint8_t byte);

    uint8_t messageId() const { return _message_id; }
    uint8_t payloadLength() const { return _payload_length; }
    const uint8_t* payload() const { return _payload; }

    uint32_t frameCount() const { return _frame_count; }
    uint16_t badCrcCount() const { return _bad_crc_count; }
    // Количество байт, отброшенных при поиске sync-последовательности.
    // Ненулевое значение означает мусор в потоке или сдвиг границы кадра.
    uint16_t resyncCount() const { return _resync_count; }
    uint16_t badLengthCount() const { return _bad_length_count; }

    void resetCounters();

private:
    enum class State : uint8_t {
        Sync1,
        Sync2,
        Id,
        Length,
        Payload,
        CrcLow,
        CrcHigh,
    };

    void restart();

    State _state;
    uint8_t _message_id;
    uint8_t _payload_length;
    uint8_t _payload_index;
    uint8_t _payload[kMaxInboundPayloadBytes];
    uint16_t _received_crc;

    uint32_t _frame_count;
    uint16_t _bad_crc_count;
    uint16_t _resync_count;
    uint16_t _bad_length_count;
};
