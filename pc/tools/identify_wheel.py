#!/usr/bin/env python3
"""Идентификация мотора: измерение коэффициентов feedforward.

Второй шаг настройки, после calibrate_encoder.py и verify_signs.py.

Что измеряется и почему
-----------------------

Регулятор колеса выдаёт::

    pwm = k_static * sign(sp) + k_velocity * sp + PID(sp - measured)

``k_static`` - PWM, при котором колесо страгивается с места, ``k_velocity`` -
наклон зависимости установившейся скорости от PWM. Обе величины измеряются,
а не подбираются: они описывают железо, а не желаемое поведение.

Смысл в том, чтобы feedforward сам выводил колесо примерно на уставку,
а ПИД правил лишь небольшой остаток. На модели мотора точный feedforward
с малым ПИ выводит колесо на уставку за один цикл, а тот же ПИ без него -
за восемьдесят с лишним.

Как измеряется
--------------

Скрипт подаёт прямой PWM в обход регуляторов: снять зависимость скорости
от PWM можно только при разорванной обратной связи. PWM повышается ступенями,
на каждой выдерживается пауза, затем усредняется установившаяся скорость.

По точкам, где колесо уже вращается, строится прямая ``pwm = a + b * скорость``.
Свободный член - ``k_static``, наклон - ``k_velocity``.

Каждое колесо измеряется отдельно и в обе стороны: моторы редко симметричны,
а трение вперёд и назад отличается почти всегда.

Робота нужно поднять над поверхностью: колёса будут вращаться.

Примеры::

    python3 identify_wheel.py --port /dev/ttyUSB0 --wheel left
    python3 identify_wheel.py --port /dev/ttyUSB0 --wheel both --apply --save
    python3 identify_wheel.py --port /dev/ttyUSB0 --wheel right --csv right.csv
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# protocol/ - общий кодек, использует и pi/, и pc/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "protocol"))

from bench import (  # noqa: E402
    WHEEL_NAMES,
    BenchLink,
    steady_state,
    wheel_index,
)
from plotting import (  # noqa: E402
    IdentificationWindow,
    require_plotting,
    set_theme,
    theme_choices,
)
from rtk_link import WHEEL_LEFT, WHEEL_RIGHT, WheelGains  # noqa: E402

#: Скорость, ниже которой колесо считается неподвижным. Ниже этого порога
#: отдельные отсчёты энкодера дают шум, а не измерение.
MOVING_THRESHOLD_RPS = 0.05



@dataclass
class Point:
    """Одна ступень: поданный PWM и установившаяся скорость."""

    pwm: int
    speed_rps: float
    spread_rps: float




@dataclass
class DirectionResult:
    """Результат идентификации в одну сторону."""

    direction: int
    points: list[Point]
    deadband_pwm: int
    k_static: float
    k_velocity: float
    residual_rps: float
    max_speed_rps: float




def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Измерить k_static и k_velocity feedforward по ступенькам PWM"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Скорость порта")
    parser.add_argument(
        "--wheel",
        default="both",
        choices=["left", "right", "both"],
        help="Какое колесо измерять (по умолчанию оба по очереди)",
    )
    parser.add_argument(
        "--max-pwm",
        type=int,
        default=200,
        help="До какого PWM подниматься (по умолчанию 200)",
    )
    parser.add_argument(
        "--step",
        type=int,
        default=10,
        help="Шаг по PWM (по умолчанию 10)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.8,
        help="Пауза на каждой ступени, секунды (по умолчанию 0.8)",
    )
    parser.add_argument(
        "--forward-only",
        action="store_true",
        help="Измерять только вперёд, не проверяя обратное направление",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Отправить измеренные коэффициенты в прошивку",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Записать коэффициенты в EEPROM (подразумевает --apply)",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Открыть окно с точками скорость-PWM и подогнанной прямой",
    )
    parser.add_argument(
        "--theme",
        default="light",
        choices=theme_choices(),
        help="Тема графиков (по умолчанию светлая)",
    )
    parser.add_argument("--csv", type=Path, help="Сохранить точки измерения в CSV")
    return parser.parse_args()




def fit_line(points: list[Point]) -> tuple[float, float, float]:
    """Метод наименьших квадратов: pwm = a + b * скорость.

    :returns: тройка (a, b, среднее отклонение по скорости).
    """

    moving = [p for p in points if abs(p.speed_rps) > MOVING_THRESHOLD_RPS]

    if len(moving) < 2:
        return 0.0, 0.0, 0.0

    speeds = [abs(p.speed_rps) for p in moving]
    pwms = [abs(float(p.pwm)) for p in moving]
    count = len(moving)

    mean_speed = sum(speeds) / count
    mean_pwm = sum(pwms) / count

    variance = sum((s - mean_speed) ** 2 for s in speeds)
    if variance < 1e-9:
        return mean_pwm, 0.0, 0.0

    slope = sum(
        (s - mean_speed) * (p - mean_pwm) for s, p in zip(speeds, pwms)
    ) / variance
    intercept = mean_pwm - slope * mean_speed

    # Остаток переводим обратно в скорость: так его можно сравнить с самой
    # скоростью и понять, насколько прямая описывает мотор.
    if abs(slope) > 1e-9:
        residual = sum(
            abs((p - (intercept + slope * s)) / slope) for s, p in zip(speeds, pwms)
        ) / count
    else:
        residual = 0.0

    return intercept, slope, residual




def measure_direction(
    link: BenchLink,
    wheel: int,
    direction: int,
    args: argparse.Namespace,
) -> DirectionResult:
    """Снять зависимость скорости от PWM в одну сторону."""

    label = "forward" if direction > 0 else "reverse"
    print(f"\ndirection = {label}")
    # pwm - команда, wheel_rps - установившаяся скорость колеса,
    # spread_rps - размах скорости на участке усреднения.
    print(f"{'pwm':>6} {'wheel_rps':>11} {'spread_rps':>12}")

    points: list[Point] = []
    deadband_pwm = 0

    for magnitude in range(0, args.max_pwm + 1, args.step):
        pwm = direction * magnitude

        left_pwm = pwm if wheel == WHEEL_LEFT else 0
        right_pwm = pwm if wheel == WHEEL_RIGHT else 0

        samples = link.hold_wheel_pwm(left_pwm, right_pwm, args.settle)

        if not samples:
            raise SystemExit(
                "Телеметрия не поступает. Проверьте порт и версию прошивки."
            )

        speed, spread = steady_state(samples, wheel)
        points.append(Point(pwm=pwm, speed_rps=speed, spread_rps=spread))

        moving = abs(speed) > MOVING_THRESHOLD_RPS
        if moving and deadband_pwm == 0:
            deadband_pwm = magnitude

        print(f"{pwm:>6} {speed:>11.3f} {spread:>12.3f}")

        # Дальше поднимать PWM бессмысленно: мотор уже упёрся в потолок.
        if len(points) >= 3:
            previous = abs(points[-2].speed_rps)
            if previous > 1.0 and abs(speed) < previous * 1.02:
                break

    link.stop(0.4)

    intercept, slope, residual = fit_line(points)
    max_speed = max((abs(p.speed_rps) for p in points), default=0.0)

    return DirectionResult(
        direction=direction,
        points=points,
        deadband_pwm=deadband_pwm,
        k_static=intercept,
        k_velocity=slope,
        residual_rps=residual,
        max_speed_rps=max_speed,
    )




def identify_wheel(
    link: BenchLink,
    wheel: int,
    args: argparse.Namespace,
) -> tuple[WheelGains, list[DirectionResult]]:
    """Измерить оба направления и свести к одному набору коэффициентов."""

    name = WHEEL_NAMES[wheel]
    print(f"\nwheel = {name}")

    directions = [1] if args.forward_only else [1, -1]
    results = [measure_direction(link, wheel, d, args) for d in directions]

    # Аппроксимация по направлениям. Единицы:
    #   deadband_pwm  - младшие разряды PWM, диапазон [-229, 229]
    #   k_static      - PWM
    #   k_velocity    - PWM на один оборот колеса в секунду
    #   max_wheel     - оборотов колеса в секунду
    #   residual      - оборотов колеса в секунду, среднее отклонение точек
    #                   от аппроксимирующей прямой
    print()
    print(f"{'direction':>10} {'deadband_pwm':>13} {'k_static_pwm':>13} "
          f"{'k_velocity_pwm_per_rps':>23} {'max_wheel_rps':>14} "
          f"{'residual_rps':>13}")
    for result in results:
        label = "forward" if result.direction > 0 else "reverse"
        print(
            f"{label:>10} {result.deadband_pwm:>13d} {result.k_static:>13.2f} "
            f"{result.k_velocity:>23.3f} {result.max_speed_rps:>14.3f} "
            f"{result.residual_rps:>13.4f}"
        )

    usable = [r for r in results if r.k_velocity > 1e-6]

    if not usable:
        raise SystemExit(
            f"Колесо {name} не вращается ни в одну сторону. Проверьте питание "
            "моторов, распиновку и verify_signs.py."
        )

    # Регулятор использует один набор коэффициентов на оба направления,
    # поэтому берём среднее. Существенная разница - повод для отдельного
    # разговора, поэтому о ней сообщаем.
    k_static = sum(r.k_static for r in usable) / len(usable)
    k_velocity = sum(r.k_velocity for r in usable) / len(usable)

    print()
    if len(usable) == 2:
        print(
            f"spread_k_static_pwm             = "
            f"{abs(usable[0].k_static - usable[1].k_static):.3f}"
        )
        print(
            f"spread_k_velocity_pwm_per_rps   = "
            f"{abs(usable[0].k_velocity - usable[1].k_velocity):.3f}"
        )

    # В прошивку уходит одно значение на оба направления: WheelController
    # не различает знак при вычислении feedforward, кроме знака k_static.
    print(f"k_static_pwm                    = {k_static:.3f}")
    print(f"k_velocity_pwm_per_rps          = {k_velocity:.3f}")
    print(
        f"max_wheel_rps                   = "
        f"{max(r.max_speed_rps for r in usable):.3f}"
    )

    return WheelGains(0.0, 0.0, 0.0, k_static, k_velocity), results




def write_csv(path: Path, per_wheel: dict[int, list[DirectionResult]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["wheel", "direction", "pwm", "speed_rps", "spread_rps"])

        for wheel, results in per_wheel.items():
            for result in results:
                for point in result.points:
                    writer.writerow(
                        [
                            WHEEL_NAMES[wheel],
                            "forward" if result.direction > 0 else "reverse",
                            point.pwm,
                            f"{point.speed_rps:.6f}",
                            f"{point.spread_rps:.6f}",
                        ]
                    )




def main() -> int:
    args = parse_args()
    set_theme(args.theme)
    apply_gains = args.apply or args.save

    wheels = (
        [WHEEL_LEFT, WHEEL_RIGHT]
        if args.wheel == "both"
        else [wheel_index(args.wheel)]
    )

    print("identify_wheel: развёртка по PWM в обход регуляторов")
    print(f"pwm_step = {args.step}, pwm_max = {args.max_pwm}, "
          f"settle_s = {args.settle}")
    print("робот должен быть поднят над поверхностью")
    time.sleep(1.0)

    measured: dict[int, WheelGains] = {}
    raw: dict[int, list[DirectionResult]] = {}

    try:
        with BenchLink(args.port, args.baud) as link:
            print("gains_before:")
            for wheel, report in sorted(link.get_gains().items()):
                print(
                    f"  {report.wheel_name:<5} kp={report.kp:.4f} ki={report.ki:.4f} "
                    f"kd={report.kd:.4f} k_static={report.k_static:.3f} "
                    f"k_velocity={report.k_velocity:.3f} "
                    f"source={'eeprom' if report.is_persisted else 'compiled'}"
                )

            link.reset(pid=True, stats=True)

            for wheel in wheels:
                gains, results = identify_wheel(link, wheel, args)
                measured[wheel] = gains
                raw[wheel] = results

            if apply_gains:
                print("\ngains_after:")
                for wheel, gains in measured.items():
                    # ПИД оставляем нулевым: он настраивается следующим шагом,
                    # поверх уже готового feedforward.
                    report = link.set_gains(wheel, gains)
                    print(
                        f"  {report.wheel_name:<5} kp={report.kp:.4f} "
                        f"ki={report.ki:.4f} kd={report.kd:.4f} "
                        f"k_static={report.k_static:.3f} "
                        f"k_velocity={report.k_velocity:.3f}"
                    )

                if args.save:
                    saved = link.save_gains()
                    persisted = all(r.is_persisted for r in saved.values())
                    print(f"  eeprom_write = {'ok' if persisted else 'failed'}")

            print()
            print(f"packets_received = {link.sequence.received}")
            print(f"packets_lost     = {link.sequence.lost} "
                  f"({100.0 * link.sequence.loss_ratio:.2f} %)")
    except KeyboardInterrupt:
        print("\nПрервано пользователем.")
        return 130

    if args.csv and raw:
        write_csv(args.csv, raw)
        print(f"csv = {args.csv}")

    if args.plot and raw and require_plotting():
        window = IdentificationWindow()
        summary = ["Измеренные коэффициенты feedforward:", ""]

        for wheel, results in raw.items():
            for result in results:
                label = (
                    f"{WHEEL_NAMES[wheel]} "
                    f"{'вперёд' if result.direction > 0 else 'назад'}"
                )
                moving = [
                    point
                    for point in result.points
                    if abs(point.speed_rps) > MOVING_THRESHOLD_RPS
                ]
                window.add_series(
                    label,
                    [abs(p.speed_rps) for p in moving],
                    [abs(float(p.pwm)) for p in moving],
                    result.k_static,
                    result.k_velocity,
                )

            gains = measured[wheel]
            summary.append(
                f"{WHEEL_NAMES[wheel]}: k_static = {gains.k_static:.2f}, "
                f"k_velocity = {gains.k_velocity:.2f}"
            )

        window.set_summary("\n".join(summary))

        print("\nОкно с графиком открыто. Закройте его, чтобы завершить.")
        window.show_blocking()

    if not apply_gains:
        print("gains_applied = no (--apply не задан)")

    return 0



if __name__ == "__main__":
    raise SystemExit(main())
