#include "frame.h"

#include <string.h>

uint16_t crc16Ccitt(const uint8_t* data, uint8_t length, uint16_t seed) {
    uint16_t crc = seed;

    for (uint8_t index = 0; index < length; ++index) {
        crc ^= static_cast<uint16_t>(data[index]) << 8;

        for (uint8_t bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000U) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }

    return crc;
}

uint8_t buildFrame(
    uint8_t* out,
    uint8_t out_capacity,
    uint8_t message_id,
    const void* payload,
    uint8_t payload_length) {
    const uint16_t total_length =
        static_cast<uint16_t>(payload_length) + kFrameOverheadBytes;

    if (out == nullptr || total_length > out_capacity) {
        return 0;
    }

    out[0] = kFrameSync1;
    out[1] = kFrameSync2;
    out[2] = message_id;
    out[3] = payload_length;

    if (payload_length > 0 && payload != nullptr) {
        memcpy(&out[4], payload, payload_length);
    }

    // CRC покрывает msg_id, len и payload, но не sync-байты.
    const uint16_t crc = crc16Ccitt(&out[2], static_cast<uint8_t>(payload_length + 2));

    out[4 + payload_length] = static_cast<uint8_t>(crc & 0xFFU);
    out[5 + payload_length] = static_cast<uint8_t>((crc >> 8) & 0xFFU);

    return static_cast<uint8_t>(total_length);
}

FrameParser::FrameParser()
    : _state(State::Sync1),
      _message_id(0),
      _payload_length(0),
      _payload_index(0),
      _payload{},
      _received_crc(0),
      _frame_count(0),
      _bad_crc_count(0),
      _resync_count(0),
      _bad_length_count(0) {}

void FrameParser::restart() {
    _state = State::Sync1;
    _payload_index = 0;
}

void FrameParser::resetCounters() {
    _frame_count = 0;
    _bad_crc_count = 0;
    _resync_count = 0;
    _bad_length_count = 0;
}

bool FrameParser::feed(uint8_t byte) {
    switch (_state) {
        case State::Sync1:
            if (byte == kFrameSync1) {
                _state = State::Sync2;
            } else {
                // Байт вне кадра: либо мусор, либо хвост кадра, который мы
                // уже отбросили. Считаем такие байты, чтобы отличить
                // «связь чистая» от «поток сдвинут».
                ++_resync_count;
            }
            return false;

        case State::Sync2:
            if (byte == kFrameSync2) {
                _state = State::Id;
            } else if (byte == kFrameSync1) {
                // Последовательность 0xAA 0xAA 0x55: остаёмся в ожидании
                // второго sync-байта, не теряя уже найденный первый.
                ++_resync_count;
            } else {
                ++_resync_count;
                _state = State::Sync1;
            }
            return false;

        case State::Id:
            _message_id = byte;
            _state = State::Length;
            return false;

        case State::Length:
            if (byte > kMaxInboundPayloadBytes) {
                ++_bad_length_count;
                restart();
                return false;
            }

            _payload_length = byte;
            _payload_index = 0;
            _state = (byte == 0) ? State::CrcLow : State::Payload;
            return false;

        case State::Payload:
            _payload[_payload_index] = byte;
            ++_payload_index;

            if (_payload_index >= _payload_length) {
                _state = State::CrcLow;
            }
            return false;

        case State::CrcLow:
            _received_crc = byte;
            _state = State::CrcHigh;
            return false;

        case State::CrcHigh: {
            _received_crc |= static_cast<uint16_t>(byte) << 8;
            restart();

            // Пересчитываем CRC по тем же байтам, что и передатчик:
            // msg_id, len, payload. Заголовок собирается на лету, чтобы
            // не держать копию всего кадра.
            uint8_t header[2] = {_message_id, _payload_length};
            uint16_t crc = crc16Ccitt(header, 2);
            crc = crc16Ccitt(_payload, _payload_length, crc);

            if (crc != _received_crc) {
                ++_bad_crc_count;
                return false;
            }

            ++_frame_count;
            return true;
        }
    }

    restart();
    return false;
}
