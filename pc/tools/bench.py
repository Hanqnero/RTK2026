#!/usr/bin/env python3
"""Стендовая обвязка поверх кодека протокола.

Скрипты настройки решают разные задачи, но одинаково работают с портом:
удерживают команду с частотой 50 Гц, собирают телеметрию, дожидаются ответа
с коэффициентами и гарантированно останавливают приводы при выходе.
Здесь это собрано один раз.

Модуль не зависит от ROS. Запускается и с ноутбука, и из контейнера
на Raspberry Pi.
"""

from __future__ import annotations

import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# protocol/ - общий кодек, использует и pi/, и pc/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "protocol"))

from transport import open_transport  # noqa: E402

from rtk_link import (  # noqa: E402
    COMMAND_FLAG_REQUEST_PID_DEBUG,
    MSG_AUTOTUNE_STATUS,
    MSG_GAINS_REPORT,
    MSG_PID_DEBUG,
    MSG_STATS,
    MSG_TELEMETRY,
    RESET_ODOMETRY,
    RESET_PID,
    RESET_STATS,
    WHEEL_LEFT,
    WHEEL_RIGHT,
    AutotuneStatus,
    FrameDecoder,
    GainsReport,
    PidDebug,
    SequenceTracker,
    Stats,
    Telemetry,
    WheelGains,
    decode_autotune_status,
    decode_gains_report,
    decode_pid_debug,
    decode_stats,
    decode_telemetry,
    pack_autotune_command,
    pack_autotune_stop,
    pack_get_gains,
    pack_save_gains,
    pack_set_gains,
    pack_velocity_command,
    pack_wheel_pwm_command,
    pack_wheel_setpoint_command,
)

#: Частота повторения команды. Прошивка глушит приводы, если команда
#: не приходила дольше kCommandTimeoutMs, поэтому её надо повторять.
COMMAND_PERIOD_S = 0.02

WHEEL_NAMES = {WHEEL_LEFT: "left", WHEEL_RIGHT: "right"}



def wheel_index(name: str) -> int:
    """Перевести имя колеса в индекс протокола."""

    normalized = name.strip().lower()

    if normalized in ("left", "l", "лев", "левое"):
        return WHEEL_LEFT
    if normalized in ("right", "r", "прав", "правое"):
        return WHEEL_RIGHT

    raise ValueError(f"неизвестное колесо: {name!r}")




@dataclass(frozen=True)
class Sample:
    """Пакет телеметрии со временем прибытия на хост."""

    time_s: float
    telemetry: Telemetry
    #: Внутренности регуляторов того же цикла, если отладка была включена.
    debug: PidDebug | None = None


    def wheel_rps(self, wheel: int) -> float:
        return (
            self.telemetry.left_wheel_rps
            if wheel == WHEEL_LEFT
            else self.telemetry.right_wheel_rps
        )



    def setpoint_rps(self, wheel: int) -> float:
        return (
            self.telemetry.left_setpoint_rps
            if wheel == WHEEL_LEFT
            else self.telemetry.right_setpoint_rps
        )



    def pwm(self, wheel: int) -> int:
        return (
            self.telemetry.left_pwm
            if wheel == WHEEL_LEFT
            else self.telemetry.right_pwm
        )



    def encoder_total(self, wheel: int) -> int:
        return (
            self.telemetry.left_encoder_total
            if wheel == WHEEL_LEFT
            else self.telemetry.right_encoder_total
        )





class BenchLink:
    """Сеанс связи со стендом.

    Использовать как контекстный менеджер: при выходе из блока приводы
    останавливаются в любом случае, включая исключение и Ctrl+C.
    """

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        reset_wait_s: float = 2.0,
    ) -> None:
        """Создать сеанс.

        :param port: путь к serial-устройству либо ``host:port`` сервера
            ``link_server.py`` на Raspberry Pi.
        """

        self._port_name = port
        self._baud = baud
        self._reset_wait_s = reset_wait_s

        self._port = None
        self._decoder = FrameDecoder()
        self.sequence = SequenceTracker()

        self.latest_stats: Stats | None = None

        #: Последний отчёт автотюнера. Обновляется в poll(), как и статистика.
        self.latest_autotune: AutotuneStatus | None = None
        self._gains: dict[int, GainsReport] = {}
        #: Кадр отладки ждёт свою телеметрию: они приходят парой, телеметрия первой.
        self._pending_debug: PidDebug | None = None

    def __enter__(self) -> "BenchLink":
        self.open()
        return self

    def __exit__(self, *exception) -> None:
        self.close()


    def open(self) -> None:
        self._port = open_transport(
            self._port_name, self._baud, self._reset_wait_s
        )
        print(f"transport = {self._port.description}")



    def close(self) -> None:
        if self._port is None:
            return

        try:
            # Останавливаем приводы независимо от причины выхода.
            self._port.write(pack_velocity_command(0.0, 0.0))
            time.sleep(0.1)
        except (OSError, SystemExit):
            pass
        finally:
            self._port.close()
            self._port = None


    def _write(self, frame: bytes) -> None:
        if self._port is None:
            raise RuntimeError("порт не открыт")
        self._port.write(frame)


    def send(self, frame: bytes) -> None:
        """Отправить готовый кадр не блокируя вызывающий код.

        В отличие от hold(), не ждёт: нужно там, где команду шлёт цикл
        с собственным темпом, например живая панель с управлением.
        """

        self._write(frame)



    def poll(self) -> list[Sample]:
        """Разобрать всё, что пришло, и вернуть новые пакеты телеметрии.

        Кадр отладки регуляторов относится к тому же циклу, что и телеметрия,
        и приходит сразу за ней, поэтому склеивается с предыдущим пакетом.
        """

        if self._port is None:
            raise RuntimeError("порт не открыт")

        waiting = self._port.in_waiting
        if not waiting:
            return []

        samples: list[Sample] = []
        now = time.monotonic()

        for message_id, payload in self._decoder.feed(self._port.read(waiting)):
            if message_id == MSG_TELEMETRY:
                telemetry = decode_telemetry(payload)
                self.sequence.update(telemetry.seq)
                samples.append(Sample(time_s=now, telemetry=telemetry))

            elif message_id == MSG_PID_DEBUG:
                debug = decode_pid_debug(payload)

                # Привязываем к телеметрии с тем же seq.
                for index, sample in enumerate(samples):
                    if sample.telemetry.seq == debug.seq:
                        samples[index] = Sample(
                            time_s=sample.time_s,
                            telemetry=sample.telemetry,
                            debug=debug,
                        )
                        break

            elif message_id == MSG_STATS:
                self.latest_stats = decode_stats(payload)

            elif message_id == MSG_GAINS_REPORT:
                report = decode_gains_report(payload)
                self._gains[report.wheel] = report

            elif message_id == MSG_AUTOTUNE_STATUS:
                self.latest_autotune = decode_autotune_status(payload)

        return samples



    def hold(self, command: bytes, seconds: float) -> list[Sample]:
        """Удерживать команду заданное время и вернуть собранную телеметрию."""

        collected: list[Sample] = []
        deadline = time.monotonic() + max(seconds, 0.0)
        next_send = 0.0

        while time.monotonic() < deadline:
            now = time.monotonic()

            if now >= next_send:
                self._write(command)
                next_send = now + COMMAND_PERIOD_S

            collected.extend(self.poll())
            time.sleep(0.002)

        return collected



    def stop(self, seconds: float = 0.5) -> list[Sample]:
        """Удерживать нулевую команду, давая приводам остановиться."""

        return self.hold(pack_velocity_command(0.0, 0.0), seconds)



    def hold_wheel_pwm(
        self,
        left_pwm: int,
        right_pwm: int,
        seconds: float,
        debug: bool = False,
    ) -> list[Sample]:
        """Удерживать прямую команду PWM в обход регуляторов."""

        flags = COMMAND_FLAG_REQUEST_PID_DEBUG if debug else 0
        return self.hold(
            pack_wheel_pwm_command(left_pwm, right_pwm, flags), seconds
        )



    def hold_wheel_setpoint(
        self,
        left_rps: float,
        right_rps: float,
        seconds: float,
        debug: bool = False,
    ) -> list[Sample]:
        """Удерживать уставку колёс в обход кинематики корпуса."""

        flags = COMMAND_FLAG_REQUEST_PID_DEBUG if debug else 0
        return self.hold(
            pack_wheel_setpoint_command(left_rps, right_rps, flags), seconds
        )



    def run_autotune(
        self,
        wheel: int,
        steady_pwm: int,
        step_pwm: int,
        wait_ms: int = 1500,
        window_rps: float = 0.05,
        pulse_ms: int = 400,
        target_accuracy: int = 90,
        tuner_period_ms: int = 100,
        timeout_s: float = 120.0,
        on_status=None,
        on_sample=None,
    ) -> AutotuneStatus:
        """Запустить релейный автотюнер прошивки и дождаться результата.

        Раскачкой и расчётом занимается PIDtuner на MCU: он обязан работать
        в темпе управляющего цикла, а по сети такой темп не выдержать.
        Здесь только запуск, ретрансляция хода и ожидание финального отчёта.

        Команда повторяется, как и любая другая: dead-man прошивки не знает,
        что колесом сейчас распоряжается тюнер, и заглушил бы приводы.

        :param on_status: вызывается на каждый отчёт о ходе автотюна.
        :param on_sample: вызывается на каждый пакет телеметрии.
        :returns: финальный отчёт с найденными коэффициентами.
        :raises SystemExit: если прошивка не завершила автотюн за timeout_s.
        """

        command = pack_autotune_command(
            wheel,
            steady_pwm,
            step_pwm,
            wait_ms,
            window_rps,
            pulse_ms,
            target_accuracy,
            tuner_period_ms,
        )

        self.latest_autotune = None
        deadline = time.monotonic() + timeout_s
        next_send = 0.0
        reported = None

        try:
            while time.monotonic() < deadline:
                now = time.monotonic()

                if now >= next_send:
                    self._write(command)
                    next_send = now + COMMAND_PERIOD_S

                for sample in self.poll():
                    if on_sample is not None:
                        on_sample(sample)

                status = self.latest_autotune
                if status is not None and status is not reported:
                    reported = status
                    if on_status is not None:
                        on_status(status)
                    if status.is_done:
                        return status

                time.sleep(0.002)
        finally:
            # Тюнер остаётся в своём режиме, пока ему не скажут иначе:
            # выход по таймауту или по Ctrl+C не должен оставить колесо
            # раскачиваться.
            self._write(pack_autotune_stop(wheel))

        raise SystemExit(
            f"автотюн не завершился за {timeout_s:.0f} с. "
            "Проверьте, что колесо действительно раскачивается: "
            "steady_pwm ниже PWM страгивания даёт неподвижное колесо."
        )



    def reset(
        self,
        odometry: bool = False,
        pid: bool = False,
        stats: bool = False,
        gains_to_default: bool = False,
    ) -> None:
        """Сбросить выбранное состояние прошивки.

        :param gains_to_default: вернуть скомпилированные коэффициенты и
            стереть запись в EEPROM. Нужно, чтобы выйти из заведомо неудачной
            настройки, не перепрошивая плату.
        """

        from rtk_link import RESET_GAINS_TO_DEFAULT, pack_reset_command

        mask = 0
        if odometry:
            mask |= RESET_ODOMETRY
        if pid:
            mask |= RESET_PID
        if stats:
            mask |= RESET_STATS
        if gains_to_default:
            mask |= RESET_GAINS_TO_DEFAULT

        if mask:
            self._write(pack_reset_command(mask))


    def _await_gains(self, timeout_s: float = 2.0) -> dict[int, GainsReport]:
        """Дождаться отчётов по обоим колёсам."""

        self._gains.clear()
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            self.poll()
            if len(self._gains) == 2:
                break
            time.sleep(0.005)

        return dict(self._gains)


    def get_gains(self, timeout_s: float = 2.0) -> dict[int, GainsReport]:
        """Запросить действующие коэффициенты обоих колёс."""

        self._write(pack_get_gains())
        gains = self._await_gains(timeout_s)

        if not gains:
            raise SystemExit(
                "Прошивка не ответила на запрос коэффициентов. "
                "Проверьте, что прошита версия с поддержкой SET_GAINS."
            )

        return gains



    def set_gains(self, wheel: int, gains: WheelGains, timeout_s: float = 2.0) -> GainsReport:
        """Задать коэффициенты одного колеса и дождаться подтверждения.

        Прошивка сбрасывает состояние этого регулятора: интеграл, накопленный
        при прежних коэффициентах, к новым отношения не имеет.
        """

        self._gains.pop(wheel, None)
        self._write(
            pack_set_gains(
                wheel, gains.kp, gains.ki, gains.kd, gains.k_static, gains.k_velocity
            )
        )

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll()
            if wheel in self._gains:
                return self._gains[wheel]
            time.sleep(0.005)

        raise SystemExit("Прошивка не подтвердила установку коэффициентов.")



    def save_gains(self, timeout_s: float = 3.0) -> dict[int, GainsReport]:
        """Записать действующие коэффициенты в EEPROM.

        Возвращённые отчёты содержат поле ``source``: по нему видно, удалась
        ли запись, или коэффициенты остались только в оперативной памяти.
        """

        self._write(pack_save_gains())
        return self._await_gains(timeout_s)




#: Колонки CSV. Порядок зафиксирован, потому что логи разных прогонов должны
#: сравниваться между собой и разбираться одним и тем же кодом.
LOG_COLUMNS = [
    "host_time_s",
    "phase",
    "seq",
    "mcu_time_ms",
    "dt_us",
    "mode",
    "flags",
    "left_setpoint_rps",
    "right_setpoint_rps",
    "left_wheel_rps",
    "right_wheel_rps",
    "left_encoder_delta",
    "right_encoder_delta",
    "left_encoder_total",
    "right_encoder_total",
    "left_pwm",
    "right_pwm",
    "odom_x_m",
    "odom_y_m",
    "odom_heading_rad",
    "current_linear_mps",
    "current_angular_rps",
    "sonar_distance_cm",
    # Внутренности регуляторов пишутся, только если запрашивались
    # отладочные кадры; иначе колонки остаются пустыми.
    "left_setpoint_dbg",
    "left_measured_dbg",
    "left_error",
    "left_p",
    "left_i",
    "left_ff",
    "left_pid_out",
    "left_out",
    "right_setpoint_dbg",
    "right_measured_dbg",
    "right_error",
    "right_p",
    "right_i",
    "right_ff",
    "right_pid_out",
    "right_out",
]



class TelemetryLogger:
    """Запись телеметрии в CSV.

    Каждая строка помечается фазой прогона, поэтому в одном файле могут
    лежать и разгон, и повороты, и остановки, а при разборе их можно
    разделить, не сверяясь с временными метками вручную.

    Логи нужны даже при наличии графиков: график показывает текущий прогон,
    а сравнивать надо с тем, что было вчера.
    """

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(LOG_COLUMNS)
        self._start = time.monotonic()
        self.rows = 0
        self.phase = ""

    def __enter__(self) -> "TelemetryLogger":
        return self

    def __exit__(self, *exception) -> None:
        self.close()


    def set_phase(self, phase: str) -> None:
        """Пометить последующие строки именем фазы прогона."""

        self.phase = phase



    def write(self, sample: "Sample") -> None:
        telemetry = sample.telemetry

        row = [
            f"{sample.time_s - self._start:.6f}",
            self.phase,
            telemetry.seq,
            telemetry.mcu_time_ms,
            telemetry.dt_us,
            telemetry.mode,
            telemetry.flags,
            f"{telemetry.left_setpoint_rps:.6f}",
            f"{telemetry.right_setpoint_rps:.6f}",
            f"{telemetry.left_wheel_rps:.6f}",
            f"{telemetry.right_wheel_rps:.6f}",
            telemetry.left_encoder_delta,
            telemetry.right_encoder_delta,
            telemetry.left_encoder_total,
            telemetry.right_encoder_total,
            telemetry.left_pwm,
            telemetry.right_pwm,
            f"{telemetry.odom_x_m:.6f}",
            f"{telemetry.odom_y_m:.6f}",
            f"{telemetry.odom_heading_rad:.6f}",
            f"{telemetry.current_linear_mps:.6f}",
            f"{telemetry.current_angular_rps:.6f}",
            telemetry.sonar_distance_cm,
        ]

        if sample.debug is None:
            row.extend([""] * 16)
        else:
            for fields in (sample.debug.left, sample.debug.right):
                row.extend(
                    [
                        f"{fields.setpoint_rps:.6f}",
                        f"{fields.measured_rps:.6f}",
                        f"{fields.error_rps:.6f}",
                        f"{fields.proportional:.6f}",
                        f"{fields.integral_term:.6f}",
                        f"{fields.feedforward:.6f}",
                        f"{fields.pid_output:.6f}",
                        f"{fields.output_pwm:.6f}",
                    ]
                )

        self._writer.writerow(row)
        self.rows += 1



    def write_all(self, samples) -> None:
        for sample in samples:
            self.write(sample)



    def flush(self) -> None:
        self._file.flush()



    def close(self) -> None:
        self._file.flush()
        self._file.close()





def steady_state(
    samples: list[Sample],
    wheel: int,
    tail_share: float = 0.4,
) -> tuple[float, float]:
    """Средняя скорость и её разброс на установившемся участке.

    :param tail_share: какую долю выборки с конца считать установившейся.
    :returns: пара (среднее, размах).
    """

    if not samples:
        return 0.0, 0.0

    tail_length = max(1, int(len(samples) * tail_share))
    speeds = [sample.wheel_rps(wheel) for sample in samples[-tail_length:]]

    mean = sum(speeds) / len(speeds)
    return mean, max(speeds) - min(speeds)


