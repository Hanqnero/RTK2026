"""
Серийный протокол между Raspberry Pi и Arduino.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Фрейм команды Pi -> Arduino (7 байт):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Байт 0:   0xA5       заголовок, байт 1
  Байт 1:   0x5A       заголовок, байт 2
  Байт 2:   left_lo    целевая скорость левого колеса, младший байт  (int16 LE)
  Байт 3:   left_hi    целевая скорость левого колеса, старший байт  (int16 LE)
  Байт 4:   right_lo   целевая скорость правого колеса, младший байт (int16 LE)
  Байт 5:   right_hi   целевая скорость правого колеса, старший байт (int16 LE)
  Байт 6:   checksum   сумма байт 0-5 по модулю 256

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Фрейм телеметрии Arduino -> Pi (11 байт):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Байт 0:   0x5A       заголовок, байт 1
  Байт 1:   0xA5       заголовок, байт 2
  Байт 2:   lc_b0      накопленные тики левого колеса, байт 0 (int32 LE, младший)
  Байт 3:   lc_b1      накопленные тики левого колеса, байт 1
  Байт 4:   lc_b2      накопленные тики левого колеса, байт 2
  Байт 5:   lc_b3      накопленные тики левого колеса, байт 3 (старший)
  Байт 6:   rc_b0      накопленные тики правого колеса, байт 0 (int32 LE, младший)
  Байт 7:   rc_b1      накопленные тики правого колеса, байт 1
  Байт 8:   rc_b2      накопленные тики правого колеса, байт 2
  Байт 9:   rc_b3      накопленные тики правого колеса, байт 3 (старший)
  Байт 10:  checksum   сумма байт 0-9 по модулю 256

Тики накапливаются с момента старта Arduino и могут быть отрицательными
(движение назад). Сброс только при перезапуске Arduino.
"""

import struct
from dataclasses import dataclass
from typing import Optional

# Заголовки пакетов — разные в каждую сторону чтобы не перепутать направление
CMD_HEADER = b"\xA5\x5A"  # Pi -> Arduino
TEL_HEADER = b"\x5A\xA5"  # Arduino -> Pi

CMD_SIZE = 7   # 2 (заголовок) + 2 (left_tps) + 2 (right_tps) + 1 (checksum)
TEL_SIZE = 11  # 2 (заголовок) + 4 (left_count) + 4 (right_count) + 1 (checksum)


class ProtocolError(Exception):
    """Base class for serial protocol errors."""


class InvalidChecksumError(ProtocolError):
    """Telemetry frame checksum mismatch — frame is corrupted."""


@dataclass
class TelemetryFrame:
    left_count: int   # накопленные тики левого колеса (int32, со знаком)
    right_count: int  # накопленные тики правого колеса (int32, со знаком)


def _checksum(data: bytes) -> int:
    """Контрольная сумма: сумма всех байт по модулю 256."""
    return sum(data) & 0xFF


def pack_command(left_tps: int, right_tps: int) -> bytes:
    """
    Упаковать команду скорости колёс в 7-байтовый фрейм.

    left_tps, right_tps — целевая скорость в тиках/сек (int16, со знаком).
    Отрицательное значение — движение назад.
    """
    payload = CMD_HEADER + struct.pack("<hh", left_tps, right_tps)
    return payload + bytes([_checksum(payload)])


def parse_telemetry(buf: bytearray) -> Optional[TelemetryFrame]:
    """
    Попытаться извлечь один телеметрический фрейм из буфера.

    Потребляет байты буфера до конца распознанного фрейма включительно.
    Возвращает None если в буфере ещё нет полного фрейма — это нормальная
    ситуация при накоплении данных, не ошибка.

    Выбрасывает InvalidChecksumError если фрейм найден но контрольная сумма
    не совпадает — это аномалия (помехи, рассинхронизация).
    """
    while len(buf) >= TEL_SIZE:
        idx = buf.find(TEL_HEADER)

        if idx < 0:
            # заголовок не найден — оставляем последний байт
            # (он может оказаться началом следующего заголовка)
            del buf[:-1]
            return None

        if idx > 0:
            # мусор перед заголовком — выбрасываем
            del buf[:idx]
            continue

        if len(buf) < TEL_SIZE:
            # заголовок есть, но фрейм ещё не накопился полностью
            return None

        frame = bytes(buf[:TEL_SIZE])
        expected = _checksum(frame[:-1])
        if frame[-1] != expected:
            raise InvalidChecksumError(
                f"Expected checksum=0x{expected:02X}, got 0x{frame[-1]:02X}"
            )

        left_count, right_count = struct.unpack("<ii", frame[2:-1])
        del buf[:TEL_SIZE]
        return TelemetryFrame(left_count=left_count, right_count=right_count)

    return None
