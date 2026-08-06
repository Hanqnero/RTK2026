#!/usr/bin/env python3
"""Живая панель состояния робота: графики скоростей, PWM и качества связи.

По умолчанию панель только слушает, поэтому её можно держать открытой рядом
с чем угодно, что уже управляет роботом: teleop, tune_wheel или ROS-мост.
Двое командующих одновременно мешали бы друг другу, поэтому режим управления
включается явно ключом ``--drive``.

С ``--drive`` роботом можно управлять прямо из окна стрелками, и отклик виден
на графике сразу же.

Что видно здесь и не видно в отдельном ступенчатом отклике
----------------------------------------------------------

* как ведут себя оба колеса одновременно, то есть насколько они расходятся;
* как выглядит ошибка слежения в движении, а не на одной ступеньке;
* деградирует ли связь: потери, ошибки CRC, срывы периода на MCU.

Отображение
-----------

Окно требует графической подсистемы. Из контейнера на Raspberry Pi по SSH
нужен проброс дисплея; без него есть ключ ``--text``, печатающий те же
величины строками.

Примеры::

    python3 monitor.py --port /dev/ttyUSB0
    python3 monitor.py --port /dev/ttyUSB0 --drive
    python3 monitor.py --port /dev/ttyUSB0 --window 40
    python3 monitor.py --port /dev/ttyUSB0 --text
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# protocol/ - общий кодек, использует и pi/, и pc/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "protocol"))

from bench import BenchLink, TelemetryLogger  # noqa: E402
from plotting import (  # noqa: E402
    LiveMonitorWindow,
    require_plotting,
    set_theme,
    theme_choices,
)
from rtk_link import (  # noqa: E402
    CONTROL_MODE_VELOCITY,
    CONTROL_MODE_WHEEL_PWM,
    CONTROL_MODE_WHEEL_SETPOINT,
    WHEEL_LEFT,
    WHEEL_RIGHT,
    describe_telemetry_flags,
    pack_wheel_pwm_command,
    pack_wheel_setpoint_command,
)

MODE_NAMES = {
    CONTROL_MODE_VELOCITY: "команда корпуса",
    CONTROL_MODE_WHEEL_SETPOINT: "уставка колёс",
    CONTROL_MODE_WHEEL_PWM: "прямой PWM",
}

SERIES_KEYS = (
    "time",
    "left_setpoint",
    "left_measured",
    "right_setpoint",
    "right_measured",
    "left_pwm",
    "right_pwm",
    "left_error",
    "right_error",
)



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Живая панель состояния робота по телеметрии Arduino"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Скорость порта")
    parser.add_argument(
        "--window",
        type=float,
        default=20.0,
        help="Ширина видимого окна графика, секунды (по умолчанию 20)",
    )
    parser.add_argument(
        "--text",
        action="store_true",
        help="Не открывать окно, печатать состояние строками",
    )
    parser.add_argument(
        "--theme",
        default="light",
        choices=theme_choices(),
        help="Тема графиков (по умолчанию светлая)",
    )
    parser.add_argument("--log", type=Path, help="Записать телеметрию в CSV")
    parser.add_argument(
        "--drive",
        action="store_true",
        help="Разрешить управление роботом стрелками прямо из окна панели",
    )
    parser.add_argument(
        "--drive-mode",
        default="pwm",
        choices=["pwm", "setpoint"],
        help=(
            "Чем управлять: прямым PWM в обход регуляторов (работает всегда) "
            "или уставкой скорости (требует настроенных коэффициентов)"
        ),
    )
    parser.add_argument(
        "--drive-pwm",
        type=int,
        default=120,
        help="Величина PWM при управлении в режиме pwm (по умолчанию 120)",
    )
    parser.add_argument(
        "--drive-rps",
        type=float,
        default=3.0,
        help="Скорость колеса в режиме setpoint, об/с (по умолчанию 3.0)",
    )
    return parser.parse_args()




def link_state(latest, silence_s: float) -> str:
    """Состояние потока телеметрии одной строкой.

    Только факт: идут пакеты или нет и стоит ли робот. Что с этим делать,
    решает оператор.
    """

    if latest is None:
        return "telemetry=none"

    if silence_s > 1.5:
        return f"telemetry=stalled silence_s={silence_s:.1f}"

    if latest.telemetry.command_timeout:
        return "telemetry=ok command=timeout"

    return "telemetry=ok"




def status_text(link: BenchLink, latest, elapsed_s: float, silence_s: float = 0.0) -> str:
    """Собрать блок с числами: качество связи и состояние прошивки."""

    note = link_state(latest, silence_s)

    if latest is None:
        return note

    lines = [
        f"elapsed_s={elapsed_s:.1f}  "
        f"mode={MODE_NAMES.get(latest.telemetry.mode, '?')}  "
        f"flags={describe_telemetry_flags(latest.telemetry.flags)}",
        "",
        f"{'wheel':>6} {'setpoint_rps':>13} {'wheel_rps':>11} {'pwm':>6} "
        f"{'encoder_total':>14}",
        f"{'left':>6} {latest.setpoint_rps(WHEEL_LEFT):>13.3f} "
        f"{latest.wheel_rps(WHEEL_LEFT):>11.3f} {latest.pwm(WHEEL_LEFT):>6d} "
        f"{latest.encoder_total(WHEEL_LEFT):>14d}",
        f"{'right':>6} {latest.setpoint_rps(WHEEL_RIGHT):>13.3f} "
        f"{latest.wheel_rps(WHEEL_RIGHT):>11.3f} {latest.pwm(WHEEL_RIGHT):>6d} "
        f"{latest.encoder_total(WHEEL_RIGHT):>14d}",
        "",
        f"packets_received={link.sequence.received} "
        f"packets_lost={link.sequence.lost} "
        f"({100.0 * link.sequence.loss_ratio:.2f} %)",
    ]

    stats = link.latest_stats
    if stats is not None:
        lines.append(
            f"mcu_dt_ms mean={stats.dt_mean_us / 1000.0:.2f} "
            f"min={stats.dt_min_us / 1000.0:.2f} "
            f"max={stats.dt_max_us / 1000.0:.2f}  "
            f"overruns={stats.overruns}  "
            f"cycle_max_ms={stats.cycle_duration_max_us / 1000.0:.2f}"
        )
        lines.append(
            f"sonar_max_ms={stats.sonar_block_max_us / 1000.0:.2f}  "
            f"tx_dropped={stats.tx_dropped}  rx_bad_crc={stats.rx_bad_crc}  "
            f"free_ram_bytes={stats.free_ram_bytes}"
        )

    lines.insert(0, note)
    lines.insert(1, "")

    return "\n".join(lines)




def run_text(
    link: BenchLink,
    args: argparse.Namespace,
    logger: TelemetryLogger | None = None,
) -> int:
    """Текстовый режим для работы без графической подсистемы."""

    start = time.monotonic()
    last_print = 0.0
    latest = None
    last_packet_at = time.monotonic()

    print("текстовый режим, Ctrl+C для выхода\n")

    while True:
        samples = link.poll()

        for sample in samples:
            latest = sample
            last_packet_at = time.monotonic()

            if logger is not None:
                logger.write(sample)

        now = time.monotonic()

        # Печатаем и при отсутствии пакетов: молчание само по себе диагноз.
        if now - last_print >= 0.5:
            last_print = now
            print("\033[2J\033[H", end="")
            print(status_text(link, latest, now - start, now - last_packet_at))

        time.sleep(0.01)




class KeyboardDriver:
    """Управление роботом с клавиатуры окна панели.

    Хранит набор зажатых клавиш и переводит его в команду колёсам. Команда
    повторяется независимо от нажатий: прошивка глушит приводы, если команда
    не приходила дольше своего таймаута.

    Два режима не равнозначны:

    ``pwm``
        Прямая команда в обход регуляторов. Работает на любом роботе,
        в том числе полностью ненастроенном, поэтому взят по умолчанию:
        первый запуск панели обычно и приходится на такой момент.

    ``setpoint``
        Уставка скорости через регуляторы. На роботе с нулевыми
        коэффициентами не даёт никакого PWM, и робот стоит, хотя команда
        доходит. Осмысленно после identify_wheel.py.
    """

    #: Коды клавиш Qt заданы числами, чтобы модуль не требовал Qt при импорте
    #: в текстовом режиме.
    KEY_LEFT = 0x01000012
    KEY_UP = 0x01000013
    KEY_RIGHT = 0x01000014
    KEY_DOWN = 0x01000015
    KEY_SPACE = 0x20
    KEY_W, KEY_A, KEY_S, KEY_D = 0x57, 0x41, 0x53, 0x44

    def __init__(self, mode: str, pwm: int, speed_rps: float) -> None:
        self.mode = mode
        self.pwm = pwm
        self.speed_rps = speed_rps
        self._pressed: set[int] = set()


    def on_key(self, key: int, pressed: bool) -> None:
        if key in (self.KEY_SPACE,):
            self._pressed.clear()
            return

        if pressed:
            self._pressed.add(key)
        else:
            self._pressed.discard(key)


    @property
    def _direction(self) -> tuple[float, float]:
        """Безразмерное направление каждого колеса в диапазоне [-2, 2]."""

        forward = bool(self._pressed & {self.KEY_UP, self.KEY_W})
        backward = bool(self._pressed & {self.KEY_DOWN, self.KEY_S})
        left = bool(self._pressed & {self.KEY_LEFT, self.KEY_A})
        right = bool(self._pressed & {self.KEY_RIGHT, self.KEY_D})

        linear = (1.0 if forward else 0.0) - (1.0 if backward else 0.0)
        # Поворот против часовой стрелки: правое колесо быстрее левого.
        turn = (1.0 if left else 0.0) - (1.0 if right else 0.0)

        return linear - turn, linear + turn


    def command(self) -> bytes:
        """Собрать кадр команды по текущему состоянию клавиш."""

        left, right = self._direction

        if self.mode == "pwm":
            return pack_wheel_pwm_command(
                int(left * self.pwm), int(right * self.pwm)
            )

        return pack_wheel_setpoint_command(
            left * self.speed_rps, right * self.speed_rps
        )


    @property
    def active(self) -> bool:
        return bool(self._pressed)


    def describe(self) -> str:
        left, right = self._direction

        if self.mode == "pwm":
            return (
                f"управление (прямой PWM): стрелки или WASD, пробел - стоп   "
                f"команда L={int(left * self.pwm):+4d} R={int(right * self.pwm):+4d}"
            )

        return (
            f"управление (уставка): стрелки или WASD, пробел - стоп   "
            f"L={left * self.speed_rps:+.2f} R={right * self.speed_rps:+.2f} об/с"
        )





def run_plot(
    link: BenchLink,
    args: argparse.Namespace,
    logger: TelemetryLogger | None = None,
) -> int:
    """Графический режим."""

    driver = (
        KeyboardDriver(args.drive_mode, args.drive_pwm, args.drive_rps)
        if args.drive
        else None
    )
    window = LiveMonitorWindow(
        window_s=args.window,
        key_handler=driver.on_key if driver is not None else None,
    )

    # Хранится вдвое больше видимого окна: так при паузе можно отмотать взглядом
    # чуть дальше, но память не растёт бесконечно за часы работы.
    capacity = max(200, int(args.window * 2 * 20))
    series: dict[str, deque[float]] = {
        key: deque(maxlen=capacity) for key in SERIES_KEYS
    }

    start = time.monotonic()
    last_redraw = 0.0
    latest = None
    last_packet_at = time.monotonic()

    print("окно открыто, Ctrl+C для выхода")

    if driver is not None:
        if driver.mode == "pwm":
            print(
                f"Управление из окна: стрелки или WASD, пробел - стоп. "
                f"Прямой PWM {args.drive_pwm} в обход регуляторов."
            )
        else:
            print(
                f"Управление из окна: стрелки или WASD, пробел - стоп. "
                f"Уставка {args.drive_rps:.1f} об/с через регуляторы."
            )
        print("кликните по окну для фокуса клавиатуры")
    else:
        print("режим наблюдения, управление: --drive")

    next_command_at = 0.0

    while not window.closed:
        # Команда повторяется постоянно, а не только при нажатии: иначе
        # сработал бы таймаут прошивки и приводы заглохли бы между нажатиями.
        if driver is not None:
            now = time.monotonic()
            if now >= next_command_at:
                link.send(driver.command())
                next_command_at = now + 0.02

        for sample in link.poll():
            latest = sample
            last_packet_at = time.monotonic()

            if logger is not None:
                logger.write(sample)

            series["time"].append(sample.time_s - start)

            for wheel, prefix in ((WHEEL_LEFT, "left"), (WHEEL_RIGHT, "right")):
                setpoint = sample.setpoint_rps(wheel)
                measured = sample.wheel_rps(wheel)

                series[f"{prefix}_setpoint"].append(setpoint)
                series[f"{prefix}_measured"].append(measured)
                series[f"{prefix}_pwm"].append(float(sample.pwm(wheel)))
                series[f"{prefix}_error"].append(setpoint - measured)

        now = time.monotonic()

        # Перерисовка ограничена 20 кадрами в секунду: телеметрия приходит
        # реже, а лишние кадры только греют процессор Raspberry Pi.
        #
        # Обновляем и при отсутствии данных: иначе пустое окно ничего
        # не объясняет, а объяснение как раз в строке состояния.
        if now - last_redraw >= 0.05:
            last_redraw = now
            status = status_text(link, latest, now - start, now - last_packet_at)
            if driver is not None:
                status = f"{driver.describe()}\n\n{status}"

            window.update(
                {key: list(values) for key, values in series.items()},
                status,
            )
        else:
            time.sleep(0.005)

    return 0




def main() -> int:
    args = parse_args()

    set_theme(args.theme)
    use_plot = not args.text and require_plotting()

    if args.drive and not use_plot:
        print(
            "Управление доступно только в графическом режиме: клавиши ловит окно.\n"
            "Для управления без окна есть teleop_keyboard.py."
        )

    if not use_plot and not args.text:
        print("Переключаюсь в текстовый режим.\n")

    logger = TelemetryLogger(args.log) if args.log else None

    try:
        with BenchLink(args.port, args.baud) as link:
            # Панель ничего не командует, но счётчики прошивки обнуляем:
            # интересно состояние этого сеанса, а не всего времени с включения.
            link.reset(stats=True)

            if logger is not None:
                logger.set_phase("monitor")

            if args.drive and args.drive_mode == "setpoint":
                # Уставка через регуляторы с нулевыми коэффициентами не даёт
                # никакого PWM: команда дойдёт, а робот не тронется.
                zero = [
                    report.wheel_name
                    for report in link.get_gains().values()
                    if report.k_velocity <= 1e-6 and report.kp <= 1e-6
                ]
                if zero:
                    print(
                        f"\nКоэффициенты колёс {', '.join(zero)} нулевые: "
                        "в режиме setpoint выход регулятора будет нулевым.\n"
                    )

            if use_plot:
                return run_plot(link, args, logger)
            return run_text(link, args, logger)
    except KeyboardInterrupt:
        print("\nОстановлено.")
        return 0
    finally:
        if logger is not None:
            logger.close()
            print(f"log = {logger.path} ({logger.rows} rows)")



if __name__ == "__main__":
    raise SystemExit(main())
