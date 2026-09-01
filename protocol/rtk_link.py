#!/usr/bin/env python3
"""Host-кодек протокола обмена с Arduino, версия 2.

Модуль намеренно не зависит ни от ROS, ни от pyserial: он работает с байтами.
Это позволяет использовать один и тот же кодек в стендовых скриптах без
запущенного ROS и проверять его обычным pytest.

Соответствие прошивке: ``arduino/include/control_protocol.h``.
Размеры структур там закреплены static_assert, здесь - проверками при импорте.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Iterator

PROTOCOL_VERSION = 2

FRAME_SYNC1 = 0xAA
FRAME_SYNC2 = 0x55
FRAME_OVERHEAD_BYTES = 6
MAX_PAYLOAD_BYTES = 255

# Host -> MCU
MSG_CMD_VELOCITY = 0x01
MSG_CMD_WHEEL_PWM = 0x02
MSG_CMD_WHEEL_SETPOINT = 0x03
MSG_SET_GAINS = 0x04
MSG_SET_CONFIG = 0x05
MSG_CMD_RESET = 0x06
MSG_SAVE_GAINS = 0x07
MSG_GET_GAINS = 0x08
MSG_CMD_AUTOTUNE = 0x09

# MCU -> Host
MSG_TELEMETRY = 0x81
MSG_PID_DEBUG = 0x82
MSG_STATS = 0x83
MSG_GAINS_REPORT = 0x84
MSG_AUTOTUNE_STATUS = 0x85

# Флаг любой команды движения: пока он взведён, прошивка добавляет к
# телеметрии кадр внутренностей регуляторов.
COMMAND_FLAG_REQUEST_PID_DEBUG = 0x01

# Индексы колёс
WHEEL_LEFT = 0
WHEEL_RIGHT = 1

# Режим управления в Telemetry.mode
CONTROL_MODE_VELOCITY = 0
CONTROL_MODE_WHEEL_SETPOINT = 1
CONTROL_MODE_WHEEL_PWM = 2

# Источник действующих коэффициентов в GainsReport.source
GAINS_SOURCE_COMPILED = 0
GAINS_SOURCE_EEPROM = 1

# Флаги TelemetryPayload.flags
# Бит 0x01 свободен: раньше означал остановку по сонару, но решение об
# остановке принимает Raspberry Pi, и прошивка больше не тормозит сама.
FLAG_COMMAND_TIMEOUT = 0x02
FLAG_PWM_SATURATED_LEFT = 0x04
FLAG_PWM_SATURATED_RIGHT = 0x08
FLAG_CYCLE_OVERRUN = 0x10

# Флаги ResetCommandPayload.mask
RESET_ODOMETRY = 0x01
RESET_PID = 0x02
RESET_STATS = 0x04
# Вернуть коэффициенты к скомпилированным и стереть запись в EEPROM.
RESET_GAINS_TO_DEFAULT = 0x08

VELOCITY_STRUCT = struct.Struct("<ffB")
RESET_STRUCT = struct.Struct("<B")
WHEEL_PWM_STRUCT = struct.Struct("<hhB")
WHEEL_SETPOINT_STRUCT = struct.Struct("<ffB")
SET_GAINS_STRUCT = struct.Struct("<Bfffff")
GAINS_REPORT_STRUCT = struct.Struct("<BfffffB")
AUTOTUNE_STRUCT = struct.Struct("<BhhHfHBH")
AUTOTUNE_STATUS_STRUCT = struct.Struct("<BBBBfff")
TELEMETRY_STRUCT = struct.Struct("<HIIhhiiffffhhfffffhBB")
PID_DEBUG_STRUCT = struct.Struct("<H16f")
STATS_STRUCT = struct.Struct("<BIIIIIIIIIHHHHHH")

assert VELOCITY_STRUCT.size == 9, "VelocityCommandPayload разошёлся с прошивкой"
assert RESET_STRUCT.size == 1, "ResetCommandPayload разошёлся с прошивкой"
assert WHEEL_PWM_STRUCT.size == 5, "WheelPwmCommandPayload разошёлся с прошивкой"
assert WHEEL_SETPOINT_STRUCT.size == 9, "WheelSetpointCommandPayload разошёлся с прошивкой"
assert SET_GAINS_STRUCT.size == 21, "SetGainsPayload разошёлся с прошивкой"
assert GAINS_REPORT_STRUCT.size == 22, "GainsReportPayload разошёлся с прошивкой"
assert AUTOTUNE_STRUCT.size == 16, "AutotuneCommandPayload разошёлся с прошивкой"
assert AUTOTUNE_STATUS_STRUCT.size == 16, "AutotuneStatusPayload разошёлся с прошивкой"
assert TELEMETRY_STRUCT.size == 66, "TelemetryPayload разошёлся с прошивкой"
assert PID_DEBUG_STRUCT.size == 66, "PidDebugPayload разошёлся с прошивкой"
assert STATS_STRUCT.size == 49, "StatsPayload разошёлся с прошивкой"



def crc16_ccitt(data: bytes, seed: int = 0xFFFF) -> int:
    """CRC16-CCITT, полином 0x1021, начальное значение 0xFFFF.

    Побитовая реализация повторяет ``crc16Ccitt`` из ``arduino/src/frame.cpp``
    буква в букву: расхождение здесь означало бы, что каждый кадр считается
    повреждённым, поэтому обе стороны проверяются одним тестовым вектором.
    """

    crc = seed

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc




def build_frame(message_id: int, payload: bytes = b"") -> bytes:
    """Собрать кадр целиком, включая sync-байты и CRC."""

    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError(
            f"payload {len(payload)} байт превышает предел кадра"
        )

    header = bytes((message_id & 0xFF, len(payload)))
    # CRC покрывает msg_id, len и payload, но не sync-байты.
    crc = crc16_ccitt(header + payload)

    return (
        bytes((FRAME_SYNC1, FRAME_SYNC2))
        + header
        + payload
        + struct.pack("<H", crc)
    )




def pack_velocity_command(
    linear_mps: float,
    angular_rps: float,
    flags: int = 0,
) -> bytes:
    """Кадр уставки скорости корпуса в соглашении ROS.

    :param linear_mps: линейная скорость вдоль оси x, м/с.
    :param angular_rps: угловая скорость вокруг оси z, рад/с,
        положительная против часовой стрелки.
    :param flags: биты ``FLAG_*`` команды, зарезервированы под шаг 3.
    """

    return build_frame(
        MSG_CMD_VELOCITY,
        VELOCITY_STRUCT.pack(
            float(linear_mps),
            float(angular_rps),
            int(flags) & 0xFF,
        ),
    )




def pack_reset_command(mask: int) -> bytes:
    """Кадр сброса одометрии, интегралов ПИД или счётчиков статистики."""

    return build_frame(MSG_CMD_RESET, RESET_STRUCT.pack(int(mask) & 0xFF))




def pack_wheel_setpoint_command(
    left_rps: float,
    right_rps: float,
    flags: int = 0,
) -> bytes:
    """Кадр уставки скорости колёс в обход кинематики корпуса.

    Позволяет крутить одно колесо при неподвижном втором, поэтому регуляторы
    настраиваются по отдельности. В режиме команды корпуса это невозможно:
    любая уставка там затрагивает оба колеса.

    :param left_rps: уставка левого колеса, оборотов в секунду.
    :param right_rps: уставка правого колеса, оборотов в секунду.
    :param flags: ``COMMAND_FLAG_REQUEST_PID_DEBUG`` для отладочных кадров.
    """

    return build_frame(
        MSG_CMD_WHEEL_SETPOINT,
        WHEEL_SETPOINT_STRUCT.pack(
            float(left_rps),
            float(right_rps),
            int(flags) & 0xFF,
        ),
    )




def pack_wheel_pwm_command(
    left_pwm: int,
    right_pwm: int,
    flags: int = 0,
) -> bytes:
    """Кадр прямой команды PWM в обход регуляторов.

    Нужен для идентификации мотора: снять зависимость установившейся скорости
    от PWM можно только при разорванной обратной связи. По этой зависимости
    вычисляются ``k_static`` и ``k_velocity`` feedforward.

    :param left_pwm: PWM левого мотора.
    :param right_pwm: PWM правого мотора.
    :param flags: ``COMMAND_FLAG_REQUEST_PID_DEBUG`` для отладочных кадров.
    """

    return build_frame(
        MSG_CMD_WHEEL_PWM,
        WHEEL_PWM_STRUCT.pack(
            int(left_pwm),
            int(right_pwm),
            int(flags) & 0xFF,
        ),
    )




def pack_set_gains(
    wheel: int,
    kp: float,
    ki: float,
    kd: float,
    k_static: float,
    k_velocity: float,
) -> bytes:
    """Кадр установки коэффициентов одного колеса.

    Прошивка отвечает кадром ``MSG_GAINS_REPORT`` и сбрасывает состояние
    этого регулятора: интеграл, накопленный при прежних коэффициентах,
    к новым отношения не имеет.

    Коэффициенты действуют немедленно, но до ``pack_save_gains`` живут только
    в оперативной памяти и теряются при выключении питания.

    :param wheel: ``WHEEL_LEFT`` или ``WHEEL_RIGHT``.
    """

    if wheel not in (WHEEL_LEFT, WHEEL_RIGHT):
        raise ValueError(f"неизвестный индекс колеса: {wheel}")

    return build_frame(
        MSG_SET_GAINS,
        SET_GAINS_STRUCT.pack(
            int(wheel),
            float(kp),
            float(ki),
            float(kd),
            float(k_static),
            float(k_velocity),
        ),
    )




def pack_save_gains() -> bytes:
    """Кадр записи действующих коэффициентов в EEPROM.

    Прошивка отвечает двумя ``MSG_GAINS_REPORT``. Поле ``source`` в ответе
    показывает, удалась ли запись: при ``GAINS_SOURCE_COMPILED`` коэффициенты
    остались только в оперативной памяти.
    """

    return build_frame(MSG_SAVE_GAINS)




def pack_get_gains() -> bytes:
    """Кадр запроса действующих коэффициентов обоих колёс."""

    return build_frame(MSG_GET_GAINS)




def pack_autotune_command(
    wheel: int,
    steady_pwm: int,
    step_pwm: int,
    wait_ms: int = 1500,
    window_rps: float = 0.05,
    pulse_ms: int = 400,
    target_accuracy: int = 90,
    tuner_period_ms: int = 100,
) -> bytes:
    """Кадр запуска релейного автотюнера на одном колесе.

    Тюнер живёт в прошивке, потому что обязан работать в темпе управляющего
    цикла: он раскачивает колесо вокруг ``steady_pwm`` и считает коэффициенты
    из периода и амплитуды автоколебаний. Регуляторы при этом отключены,
    поэтому робот должен быть поднят, как и для прямого PWM.

    :param wheel: ``WHEEL_LEFT`` или ``WHEEL_RIGHT``.
    :param steady_pwm: базовый PWM. Обязан быть выше PWM страгивания,
        иначе колесо стоит в мёртвой зоне и автоколебаний не возникает.
    :param step_pwm: амплитуда релейной ступеньки. Ноль останавливает
        автотюн и возвращает прошивку в режим уставки.
    :param wait_ms: сколько ждать установившейся скорости перед раскачкой.
    :param window_rps: порог скорости изменения, ниже которого система
        считается устоявшейся.
    :param pulse_ms: длительность первого импульса раскачки.
    :param target_accuracy: совпадение соседних периодов в процентах,
        при котором автотюн завершается.
    :param tuner_period_ms: период итерации тюнера. Намеренно медленнее
        управляющего цикла: у скоростного контура мотора почти нет
        запаздывания, и на 25 мс реле переключается раньше, чем скорость
        уйдёт от средней линии, отчего амплитуда автоколебаний вырождается,
        а расчётный Ku улетает в тысячи.
    """

    return build_frame(
        MSG_CMD_AUTOTUNE,
        AUTOTUNE_STRUCT.pack(
            int(wheel) & 0xFF,
            int(steady_pwm),
            int(step_pwm),
            int(wait_ms) & 0xFFFF,
            float(window_rps),
            int(pulse_ms) & 0xFFFF,
            int(target_accuracy) & 0xFF,
            int(tuner_period_ms) & 0xFFFF,
        ),
    )




def pack_autotune_stop(wheel: int = WHEEL_LEFT) -> bytes:
    """Кадр остановки автотюна: нулевая ступенька раскачки."""

    return pack_autotune_command(wheel, steady_pwm=0, step_pwm=0)




@dataclass(frozen=True, slots=True)
class Telemetry:
    """Разобранный ``TelemetryPayload``."""

    seq: int
    mcu_time_ms: int
    dt_us: int

    left_encoder_delta: int
    right_encoder_delta: int

    #: Накопленные отсчёты с запуска прошивки. Дублируют дельты намеренно:
    #: сумма дельт на хосте разъезжается при потере пакета, а счётчик - нет.
    left_encoder_total: int
    right_encoder_total: int

    left_wheel_rps: float
    right_wheel_rps: float

    #: Действующие уставки колёс. Без них измеренную скорость не с чем
    #: сравнивать: в режимах, отличных от команды корпуса, уставка колеса
    #: не выводится из команды однозначно.
    left_setpoint_rps: float
    right_setpoint_rps: float

    left_pwm: int
    right_pwm: int

    odom_x_m: float
    odom_y_m: float
    odom_heading_rad: float

    current_linear_mps: float
    current_angular_rps: float

    sonar_distance_cm: int
    flags: int

    #: Одна из констант ``CONTROL_MODE_*``.
    mode: int

    @property
    def dt_s(self) -> float:
        """Фактический интервал управляющего цикла в секундах."""
        return self.dt_us * 1e-6

    @property
    def command_timeout(self) -> bool:
        return bool(self.flags & FLAG_COMMAND_TIMEOUT)

    @property
    def pwm_saturated(self) -> bool:
        return bool(
            self.flags & (FLAG_PWM_SATURATED_LEFT | FLAG_PWM_SATURATED_RIGHT)
        )

    @property
    def cycle_overrun(self) -> bool:
        return bool(self.flags & FLAG_CYCLE_OVERRUN)




@dataclass(frozen=True, slots=True)
class Stats:
    """Разобранный ``StatsPayload``.

    Поля ``dt_*``, ``cycle_duration_max_us`` и ``sonar_block_max_us``
    относятся к окну между двумя отправками статистики. Остальные счётчики
    накопительные от старта прошивки, шестнадцатибитные переполняются.
    """

    protocol_version: int
    uptime_ms: int
    control_cycles: int

    dt_min_us: int
    dt_max_us: int
    dt_mean_us: int
    cycle_duration_max_us: int
    sonar_block_max_us: int

    tx_frames: int
    rx_frames: int

    tx_dropped: int
    rx_bad_crc: int
    rx_resync: int
    rx_bad_length: int
    overruns: int

    free_ram_bytes: int




def decode_telemetry(payload: bytes) -> Telemetry:
    """Разобрать полезную нагрузку кадра ``MSG_TELEMETRY``."""

    return Telemetry(*TELEMETRY_STRUCT.unpack(payload))




def decode_stats(payload: bytes) -> Stats:
    """Разобрать полезную нагрузку кадра ``MSG_STATS``."""

    return Stats(*STATS_STRUCT.unpack(payload))





@dataclass(frozen=True, slots=True)
class WheelGains:
    """Коэффициенты регулятора одного колеса."""

    kp: float
    ki: float
    kd: float
    #: PWM, необходимый для страгивания колеса с места.
    k_static: float
    #: PWM на один оборот в секунду в установившемся режиме.
    k_velocity: float




@dataclass(frozen=True, slots=True)
class AutotuneStatus:
    """Разобранный ``AutotuneStatusPayload``.

    Приходит раз в четверть секунды, пока автотюн работает, и один раз
    с ``done=1`` по завершении.
    """

    wheel: int
    #: Этап PIDtuner - 1 стабилизация, 2 импульс, 3 анализ автоколебаний.
    state: int
    #: Совпадение соседних периодов автоколебаний, проценты.
    accuracy: int
    #: Ненулевой, когда коэффициенты окончательные.
    done: int

    kp: float
    ki: float
    kd: float

    @property
    def wheel_name(self) -> str:
        return "left" if self.wheel == WHEEL_LEFT else "right"

    @property
    def is_done(self) -> bool:
        return self.done != 0

    @property
    def stage_name(self) -> str:
        """Человекочитаемый этап.

        Нулевой этап в отчёт не попадает: тюнер уходит с него в первом же
        вызове ``compute()``.
        """

        return {
            1: "стабилизация",
            2: "импульс раскачки",
            3: "анализ автоколебаний",
        }.get(self.state, f"этап {self.state}")


@dataclass(frozen=True, slots=True)
class GainsReport:
    """Разобранный ``GainsReportPayload``."""

    wheel: int
    kp: float
    ki: float
    kd: float
    k_static: float
    k_velocity: float
    #: ``GAINS_SOURCE_EEPROM`` или ``GAINS_SOURCE_COMPILED``.
    source: int

    @property
    def gains(self) -> WheelGains:
        return WheelGains(
            self.kp, self.ki, self.kd, self.k_static, self.k_velocity
        )

    @property
    def wheel_name(self) -> str:
        return "left" if self.wheel == WHEEL_LEFT else "right"

    @property
    def is_persisted(self) -> bool:
        """Коэффициенты переживут выключение питания."""
        return self.source == GAINS_SOURCE_EEPROM




@dataclass(frozen=True, slots=True)
class WheelDebug:
    """Внутренности регулятора одного колеса за один цикл."""

    setpoint_rps: float
    measured_rps: float
    error_rps: float
    proportional: float
    integral_term: float
    feedforward: float
    pid_output: float
    output_pwm: float

    @property
    def derivative(self) -> float:
        """Дифференциальная составляющая выхода регулятора.

        Отдельным полем не передаётся: uPID не отдаёт её наружу, но она
        однозначно восстанавливается как остаток выхода регулятора после
        вычета пропорциональной и интегральной составляющих.
        """
        return self.pid_output - self.proportional - self.integral_term

    @property
    def feedforward_share(self) -> float:
        """Доля feedforward в итоговой команде.

        Хорошо настроенный контур держит её высокой: значит, ПИД правит
        небольшой остаток, а не тянет весь сигнал в одиночку.
        """
        if self.output_pwm == 0.0:
            return 0.0
        return self.feedforward / self.output_pwm




@dataclass(frozen=True, slots=True)
class PidDebug:
    """Разобранный ``PidDebugPayload`` обоих колёс."""

    #: Совпадает с seq телеметрии того же цикла.
    seq: int
    left: WheelDebug
    right: WheelDebug




def decode_gains_report(payload: bytes) -> GainsReport:
    """Разобрать полезную нагрузку кадра ``MSG_GAINS_REPORT``."""

    return GainsReport(*GAINS_REPORT_STRUCT.unpack(payload))




def decode_autotune_status(payload: bytes) -> AutotuneStatus:
    """Разобрать полезную нагрузку кадра ``MSG_AUTOTUNE_STATUS``."""

    return AutotuneStatus(*AUTOTUNE_STATUS_STRUCT.unpack(payload))




def decode_pid_debug(payload: bytes) -> PidDebug:
    """Разобрать полезную нагрузку кадра ``MSG_PID_DEBUG``."""

    values = PID_DEBUG_STRUCT.unpack(payload)

    return PidDebug(
        seq=values[0],
        left=WheelDebug(*values[1:9]),
        right=WheelDebug(*values[9:17]),
    )




class FrameDecoder:
    """Побайтовый разборщик входящего потока кадров.

    Повторяет конечный автомат ``FrameParser`` из прошивки. В отличие от
    протокола v1, где первый байт буфера считался началом пакета, здесь
    синхронизация восстанавливается автоматически.

    Восстановление не мгновенное: сдвинутое поле длины заставляет разборщик
    заглотить начало следующего кадра, поэтому один потерянный байт стоит
    до двух кадров. Это цена кадрирования по sync-слову без байт-стаффинга.
    """

    _STATE_SYNC1 = 0
    _STATE_SYNC2 = 1
    _STATE_ID = 2
    _STATE_LENGTH = 3
    _STATE_PAYLOAD = 4
    _STATE_CRC_LOW = 5
    _STATE_CRC_HIGH = 6

    def __init__(self, max_payload_bytes: int = MAX_PAYLOAD_BYTES) -> None:
        self._max_payload_bytes = max_payload_bytes

        self._state = self._STATE_SYNC1
        self._message_id = 0
        self._payload_length = 0
        self._payload = bytearray()
        self._received_crc = 0

        self.frame_count = 0
        self.bad_crc_count = 0
        # Количество байт, отброшенных при поиске sync-последовательности.
        self.resync_count = 0
        self.bad_length_count = 0


    def feed(self, data: bytes) -> Iterator[tuple[int, bytes]]:
        """Скормить очередную порцию байт и получить готовые кадры.

        :returns: генератор пар ``(message_id, payload)``.
        """

        for byte in data:
            message = self._feed_byte(byte)

            if message is not None:
                yield message


    def _feed_byte(self, byte: int) -> tuple[int, bytes] | None:
        if self._state == self._STATE_SYNC1:
            if byte == FRAME_SYNC1:
                self._state = self._STATE_SYNC2
            else:
                self.resync_count += 1
            return None

        if self._state == self._STATE_SYNC2:
            if byte == FRAME_SYNC2:
                self._state = self._STATE_ID
            elif byte == FRAME_SYNC1:
                # Последовательность AA AA 55: остаёмся в ожидании второго
                # sync-байта, не теряя уже найденный первый.
                self.resync_count += 1
            else:
                self.resync_count += 1
                self._state = self._STATE_SYNC1
            return None

        if self._state == self._STATE_ID:
            self._message_id = byte
            self._state = self._STATE_LENGTH
            return None

        if self._state == self._STATE_LENGTH:
            if byte > self._max_payload_bytes:
                self.bad_length_count += 1
                self._state = self._STATE_SYNC1
                return None

            self._payload_length = byte
            self._payload = bytearray()
            self._state = (
                self._STATE_CRC_LOW if byte == 0 else self._STATE_PAYLOAD
            )
            return None

        if self._state == self._STATE_PAYLOAD:
            self._payload.append(byte)

            if len(self._payload) >= self._payload_length:
                self._state = self._STATE_CRC_LOW
            return None

        if self._state == self._STATE_CRC_LOW:
            self._received_crc = byte
            self._state = self._STATE_CRC_HIGH
            return None

        # _STATE_CRC_HIGH
        self._received_crc |= byte << 8
        self._state = self._STATE_SYNC1

        header = bytes((self._message_id, self._payload_length))
        if crc16_ccitt(header + bytes(self._payload)) != self._received_crc:
            self.bad_crc_count += 1
            return None

        self.frame_count += 1
        return (self._message_id, bytes(self._payload))




class SequenceTracker:
    """Учёт потерь телеметрии по полю ``seq``.

    Прошивка нумерует пакеты подряд, поэтому разрыв номеров означает именно
    потерю, а не задержку. Без этого потеря пакета выглядела бы на хосте
    просто как менее частая телеметрия.
    """

    def __init__(self) -> None:
        self.received = 0
        self.lost = 0
        self.reordered = 0
        self._last_seq: int | None = None


    def update(self, seq: int) -> int:
        """Учесть пакет и вернуть число потерянных перед ним пакетов."""

        self.received += 1

        if self._last_seq is None:
            self._last_seq = seq
            return 0

        # Разность по модулю 2^16: счётчик прошивки переполняется.
        gap = (seq - self._last_seq) & 0xFFFF

        if gap == 0:
            self.reordered += 1
            return 0

        # Большой gap означает не потерю тысяч пакетов, а перезапуск MCU
        # или доставку пакета не по порядку. Считать это потерями нельзя:
        # статистика стала бы бессмысленной.
        if gap > 0x8000:
            self.reordered += 1
            return 0

        self._last_seq = seq
        missing = gap - 1
        self.lost += missing
        return missing


    @property
    def loss_ratio(self) -> float:
        """Доля потерянных пакетов от ожидавшихся."""

        expected = self.received + self.lost

        if expected == 0:
            return 0.0

        return self.lost / expected


    def reset(self) -> None:
        self.received = 0
        self.lost = 0
        self.reordered = 0
        self._last_seq = None





def describe_telemetry_flags(flags: int) -> str:
    """Человекочитаемый список взведённых флагов телеметрии."""

    names = []

    if flags & FLAG_COMMAND_TIMEOUT:
        names.append("cmd_timeout")
    if flags & FLAG_PWM_SATURATED_LEFT:
        names.append("sat_left")
    if flags & FLAG_PWM_SATURATED_RIGHT:
        names.append("sat_right")
    if flags & FLAG_CYCLE_OVERRUN:
        names.append("overrun")

    return ",".join(names) if names else "-"

