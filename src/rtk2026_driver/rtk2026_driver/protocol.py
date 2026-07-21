"""
! Бинарный протокол обмена между Raspberry Pi и Arduino (пока без никаких байтовых проверок)

Соответствие Arduino (вот прямо щас такой):

.. code-block:: cpp

    struct __attribute__((packed)) ControlPacket {
        float target_linear_mps;
        float target_angular_rps;
        uint8_t debug_raw_encoder;
    };

    struct __attribute__((packed)) TelemetryPacket {
        float odom_x_m;
        float odom_y_m;
        float odom_heading_rad;
        int32_t raw_left_encoder_delta;
        int32_t raw_right_encoder_delta;
        int16_t left_pwm;
        int16_t right_pwm;
        float current_linear_mps;
        float current_angular_rps;
    };

~ Arduino Mega использует little-endian, float размером 4 байта.
~ Форматы Python struct также принудительно задаются как little-endian.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

"""
 ? Символ "<" означает:
 - little-endian (самый младший байт (наименее значащий) записывается по меньшему адресу, а самый старший — по большему)
 - стандартный размер типов
 - отсутствие автоматического выравнивания между полями
 "f" — 32-битный float.
 "B" — 8-битное беззнаковое целое.
 ? Итоговый размер:
   4 + 4 + 1 = 9 байт.
"""

COMMAND_STRUCT = struct.Struct("<ffB")


# Поля:
#   f — odom_x_m;
#   f — odom_y_m;
#   f — odom_heading_rad;
#   i — raw_left_encoder_delta, int32;
#   i — raw_right_encoder_delta, int32;
#   h — left_pwm, int16;
#   h — right_pwm, int16;
#   f — current_linear_mps;
#   f — current_angular_rps.
#
# Итог:
#   4 + 4 + 4 + 4 + 4 + 2 + 2 + 4 + 4 = 32 байта.
TELEMETRY_STRUCT = struct.Struct("<fffiihhff")


COMMAND_PACKET_SIZE = COMMAND_STRUCT.size
TELEMETRY_PACKET_SIZE = TELEMETRY_STRUCT.size


# Эти проверки выполняются при импорте модуля.
# Если формат случайно изменят и размер перестанет совпадать с Arduino,
# программа завершится сразу, а не будет молча интерпретировать неправильные данные.
assert COMMAND_PACKET_SIZE == 9
assert TELEMETRY_PACKET_SIZE == 32


@dataclass(frozen=True, slots=True)
class TelemetryPacket:
    """
    Декодированный пакет телеметрии Arduino.

    frozen=True:
        После создания объект нельзя изменить. Это предотвращает случайное
        изменение измерения после разбора serial-пакета.

    slots=True:
        Python не создаёт отдельный __dict__ для каждого объекта. Для потока
        из десятков пакетов в секунду это немного уменьшает накладные расходы.
    """

    odom_x_m: float
    odom_y_m: float
    odom_heading_rad: float

    raw_left_encoder_delta: int
    raw_right_encoder_delta: int

    left_pwm: int
    right_pwm: int

    current_linear_mps: float
    current_angular_rps: float


def pack_command(
    linear_mps: float,
    angular_rps: float,
    debug_raw_encoder: bool = False,
) -> bytes:
    """
    Сформировать ровно один ControlPacket для Arduino.

    ~ linear_mps:
        Требуемая линейная скорость центра робота, м/с.

        Положительное значение означает движение вперёд при условии,
        что знаки моторов и энкодеров правильно настроены в прошивке.

    ~ angular_rps:
        Требуемая угловая скорость, рад/с.

        В ROS положительный angular.z обычно означает вращение против
        часовой стрелки вокруг положительной вертикальной оси Z.

    ~ debug_raw_encoder:
        Если True, Arduino будет добавлять сырые дельты энкодеров
        в TelemetryPacket.

        Одометрия и текущая скорость передаются независимо от этого флага.
        Для обычной работы SLAM этот флаг не требуется.

    ? output:
        Ровно 9 байт в формате, который ожидает Arduino.
    """

    return COMMAND_STRUCT.pack(
        float(linear_mps),
        float(angular_rps),
        1 if debug_raw_encoder else 0, )


def pop_telemetry_packet(buffer: bytearray) -> TelemetryPacket | None:
    """
    Извлечь один полный TelemetryPacket из начала накопительного буфера.

    Serial.read() не обязан возвращать целый пакет. Возможны варианты:

        первое чтение:  7 байт;
        второе чтение: 15 байт;
        третье чтение: 10 байт.

    Вместе это 32 байта, то есть один TelemetryPacket.

    Поэтому байты сначала накапливаются в bytearray. Пакет разбирается только
    тогда, когда в буфере есть не менее 32 байт.

    Важно:
        Здесь отсутствует поиск начала фрейма. Предполагается, что первый байт
        буфера всегда является первым байтом TelemetryPacket.
    """

    if len(buffer) < TELEMETRY_PACKET_SIZE:
        return None

    # Копируем первые 32 байта в неизменяемый bytes.
    raw_packet = bytes(buffer[:TELEMETRY_PACKET_SIZE])

    # Удаляем разобранные байты из начала накопительного буфера.
    del buffer[:TELEMETRY_PACKET_SIZE]

    values = TELEMETRY_STRUCT.unpack(raw_packet)

    return TelemetryPacket(
        odom_x_m=float(values[0]),
        odom_y_m=float(values[1]),
        odom_heading_rad=float(values[2]),
        raw_left_encoder_delta=int(values[3]),
        raw_right_encoder_delta=int(values[4]),
        left_pwm=int(values[5]),
        right_pwm=int(values[6]),
        current_linear_mps=float(values[7]),
        current_angular_rps=float(values[8]),
    )
