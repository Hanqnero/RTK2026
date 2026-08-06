#!/usr/bin/env python3
"""Проверка адекватности энкодеров: знаки, отклик, симметрия колёс.

Первое, что делают на новом роботе. Пока не доказано, что энкодеры считают
верно, всё дальнейшее бессмысленно: и одометрия, и feedforward, и ПИД
опираются на их показания, и ошибка в знаке или масштабе тихо разъедется
по всей цепочке.

Что проверяется
---------------

Инструмент крутит каждое колесо в обе стороны и показывает, сколько отсчётов
набрал каждый энкодер, какая была установившаяся скорость и её разброс.
Выводы о том, что из этого норма, инструмент не делает: он измеряет.

Величины, по которым обычно судят:

* отсчёты проверяемого колеса - отвечает ли энкодер вообще;
* знак отсчётов относительно направления команды - верны ли флаги
  ``k*Reverse`` в ``motor_interface.h``;
* отсчёты второго колеса - нет ли перекрёстного вращения;
* отношение колёс - насколько моторы различаются.

Режимы
------

``--drive`` (по умолчанию)
    Ручная проверка: вы крутите робота с клавиатуры и смотрите, как отзываются
    энкодеры. Полезно, когда надо покрутить колесо рукой или поймать дребезг
    на конкретном участке.

``--auto``
    Автоматическая последовательность: каждое колесо в обе стороны, с выводом
    вердикта по каждому пункту.

Робота нужно поднять над поверхностью: колёса будут вращаться.

Примеры::

    python3 check_encoders.py --port /dev/ttyUSB0
    python3 check_encoders.py --port /dev/ttyUSB0 --auto --log encoders.csv
    python3 check_encoders.py --port /dev/ttyUSB0 --auto --no-plot
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# protocol/ - общий кодек, использует и pi/, и pc/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "protocol"))

from bench import (  # noqa: E402
    WHEEL_NAMES,
    BenchLink,
    Sample,
    TelemetryLogger,
    steady_state,
)
from plotting import (  # noqa: E402
    LiveMonitorWindow,
    require_plotting,
    set_theme,
    theme_choices,
)
from rtk_link import WHEEL_LEFT, WHEEL_RIGHT  # noqa: E402

#: Ниже этого модуля скорости колесо считается неподвижным.
MOVING_THRESHOLD_RPS = 0.05

#: PWM автоматической проверки. Заведомо выше мёртвой зоны любого варианта
#: JGB37-520, но далеко от насыщения.
AUTO_PWM = 120

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



@dataclass
class WheelCase:
    """Результат одной проверки: колесо в одном направлении."""

    wheel: int
    direction: int
    counts: int
    other_counts: int
    speed_rps: float
    spread_rps: float
    samples: int




def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Проверить, что энкодеры считают верно"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Скорость порта")

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--drive",
        action="store_true",
        help="Ручная проверка: управление с клавиатуры и живые графики",
    )
    mode.add_argument(
        "--auto",
        action="store_true",
        help="Автоматическая последовательность с вердиктом по каждому пункту",
    )

    parser.add_argument(
        "--pwm",
        type=int,
        default=AUTO_PWM,
        help=f"Величина PWM (по умолчанию {AUTO_PWM})",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=1.5,
        help="Длительность каждой фазы в режиме --auto, секунды",
    )
    parser.add_argument("--log", type=Path, help="Записать телеметрию в CSV")
    parser.add_argument(
        "--no-plot", action="store_true", help="Не открывать окно графиков"
    )
    parser.add_argument(
        "--theme",
        default="light",
        choices=theme_choices(),
        help="Тема графиков (по умолчанию светлая)",
    )
    return parser.parse_args()




def run_case(
    link: BenchLink,
    wheel: int,
    direction: int,
    pwm: int,
    hold_s: float,
    logger: TelemetryLogger | None,
) -> WheelCase:
    """Покрутить одно колесо в одну сторону и снять показания."""

    name = WHEEL_NAMES[wheel]
    label = "вперёд" if direction > 0 else "назад"

    link.stop(0.4)
    baseline = link.hold_wheel_pwm(0, 0, 0.3)

    if not baseline:
        raise SystemExit(
            "Телеметрия не поступает. Проверьте порт и версию прошивки."
        )

    start_left = baseline[-1].encoder_total(WHEEL_LEFT)
    start_right = baseline[-1].encoder_total(WHEEL_RIGHT)

    if logger is not None:
        logger.set_phase(f"{name}_{label}")

    command_pwm = direction * pwm
    samples = link.hold_wheel_pwm(
        command_pwm if wheel == WHEEL_LEFT else 0,
        command_pwm if wheel == WHEEL_RIGHT else 0,
        hold_s,
    )

    if logger is not None:
        logger.write_all(samples)

    link.stop(0.3)

    end_left = samples[-1].encoder_total(WHEEL_LEFT)
    end_right = samples[-1].encoder_total(WHEEL_RIGHT)

    left_counts = end_left - start_left
    right_counts = end_right - start_right

    speed, spread = steady_state(samples, wheel)

    return WheelCase(
        wheel=wheel,
        direction=direction,
        counts=left_counts if wheel == WHEEL_LEFT else right_counts,
        other_counts=right_counts if wheel == WHEEL_LEFT else left_counts,
        speed_rps=speed,
        spread_rps=spread,
        samples=len(samples),
    )




def report_cases(cases: list[WheelCase]) -> None:
    """Напечатать измерения по всем проверкам.

    Только числа: сколько отсчётов набрало проверяемое колесо, сколько
    набрало второе, какая была установившаяся скорость и её разброс.
    Что из этого считать нормой - решает тот, кто проводит проверку.
    """

    # counts        - прирост накопленного счётчика проверяемого колеса
    # other_counts  - прирост счётчика второго колеса за то же время
    # wheel_rps     - установившаяся скорость проверяемого колеса
    # spread_rps    - размах скорости на участке усреднения
    print()
    print(f"{'wheel':>6} {'direction':>10} {'counts':>9} {'other_counts':>13} "
          f"{'wheel_rps':>11} {'spread_rps':>12}")

    for case in cases:
        label = "forward" if case.direction > 0 else "reverse"
        print(
            f"{WHEEL_NAMES[case.wheel]:>6} {label:>10} "
            f"{case.counts:>9d} {case.other_counts:>13d} "
            f"{case.speed_rps:>11.3f} {case.spread_rps:>12.3f}"
        )

    # Отношение колёс: иначе его пришлось бы считать вручную по таблице.
    print()
    for direction in (1, -1):
        pair = [c for c in cases if c.direction == direction]
        if len(pair) != 2:
            continue

        magnitudes = [abs(c.counts) for c in pair]
        if min(magnitudes) == 0:
            continue

        label = "forward" if direction > 0 else "reverse"
        print(
            f"wheel_count_ratio_{label} = "
            f"{min(magnitudes) / max(magnitudes):.4f}"
        )




def run_auto(
    link: BenchLink,
    args: argparse.Namespace,
    logger: TelemetryLogger | None,
) -> int:
    """Автоматическая последовательность: каждое колесо в обе стороны."""

    print(f"pwm = {args.pwm}, hold_s = {args.hold:.2f}")

    cases: list[WheelCase] = []

    for wheel in (WHEEL_LEFT, WHEEL_RIGHT):
        for direction in (1, -1):
            cases.append(
                run_case(link, wheel, direction, args.pwm, args.hold, logger)
            )

    report_cases(cases)

    print()
    print(f"packets_received = {link.sequence.received}")
    print(f"packets_lost     = {link.sequence.lost} "
          f"({100.0 * link.sequence.loss_ratio:.2f} %)")
    return 0




def run_drive(
    link: BenchLink,
    args: argparse.Namespace,
    logger: TelemetryLogger | None,
) -> int:
    """Ручная проверка: управление с клавиатуры и живые графики."""

    # Импорт здесь: управление имеет смысл только вместе с окном.
    from monitor import KeyboardDriver, status_text

    driver = KeyboardDriver("pwm", args.pwm, 3.0)
    window = None

    if not args.no_plot and require_plotting():
        window = LiveMonitorWindow(
            window_s=20.0,
            title="Проверка энкодеров",
            key_handler=driver.on_key,
        )
        print(f"стрелки/WASD - pwm {args.pwm}, пробел - стоп")
    else:
        print("клавиши ловит окно графиков; используйте --auto")
        return 1

    if logger is not None:
        logger.set_phase("drive")

    capacity = 800
    series: dict[str, deque[float]] = {
        key: deque(maxlen=capacity) for key in SERIES_KEYS
    }
    deltas = {WHEEL_LEFT: deque(maxlen=120), WHEEL_RIGHT: deque(maxlen=120)}

    start = time.monotonic()
    latest: Sample | None = None
    last_packet_at = start
    next_command_at = 0.0
    last_redraw = 0.0

    while not window.closed:
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

            deltas[WHEEL_LEFT].append(sample.telemetry.left_encoder_delta)
            deltas[WHEEL_RIGHT].append(sample.telemetry.right_encoder_delta)

        if now - last_redraw >= 0.05:
            last_redraw = now

            status = driver.describe()
            if latest is not None:
                status += "\n\n" + encoder_status(latest, deltas)
                status += "\n\n" + status_text(
                    link, latest, now - start, now - last_packet_at
                )
            else:
                status += "\n\nТелеметрия не поступает."

            window.update(
                {key: list(values) for key, values in series.items()}, status
            )
        else:
            time.sleep(0.005)

    if logger is not None:
        logger.flush()

    return 0




def encoder_status(latest: Sample, deltas: dict[int, deque]) -> str:
    """Блок именно про энкодеры: дельты, счётчики и признак молчания."""

    lines = []

    for wheel, prefix in ((WHEEL_LEFT, "левый "), (WHEEL_RIGHT, "правый")):
        recent = list(deltas[wheel])
        silent = recent and all(value == 0 for value in recent)
        note = "  <- молчит" if silent else ""

        lines.append(
            f"энкодер {prefix}: дельта {latest.telemetry.left_encoder_delta if wheel == WHEEL_LEFT else latest.telemetry.right_encoder_delta:+5d}   "
            f"счётчик {latest.encoder_total(wheel):+9d}   "
            f"скорость {latest.wheel_rps(wheel):+6.2f} об/с{note}"
        )

    left_recent = sum(abs(v) for v in deltas[WHEEL_LEFT])
    right_recent = sum(abs(v) for v in deltas[WHEEL_RIGHT])

    if left_recent and right_recent:
        ratio = min(left_recent, right_recent) / max(left_recent, right_recent)
        lines.append(f"симметрия за последние секунды: {100.0 * ratio:.0f} %")

    return "\n".join(lines)




def main() -> int:
    args = parse_args()
    set_theme(args.theme)

    # Ручной режим взят по умолчанию: первое, что хочется на новом роботе -
    # покрутить колесо и увидеть, что счётчик шевелится.
    automatic = args.auto

    print("check_encoders: робот должен быть поднят над поверхностью")
    time.sleep(1.0)

    logger = TelemetryLogger(args.log) if args.log else None

    try:
        with BenchLink(args.port, args.baud) as link:
            link.reset(odometry=True, pid=True, stats=True)

            if automatic:
                return run_auto(link, args, logger)
            return run_drive(link, args, logger)
    except KeyboardInterrupt:
        print("\nПрервано пользователем.")
        return 130
    finally:
        if logger is not None:
            logger.close()
            print(f"log = {logger.path} ({logger.rows} rows)")



if __name__ == "__main__":
    raise SystemExit(main())
