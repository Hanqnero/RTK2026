#!/usr/bin/env python3
"""Маршрутные тесты одометрии: сравнение желаемого пути с энкодерным.

Последний шаг проверки привода, после настройки регуляторов.

Что ловят эти тесты и не ловят предыдущие
------------------------------------------

Графики отдельного колеса показывают, что регулятор держит заданную скорость.
Но одометрия строится не из скорости колеса, а из неё вместе с двумя
геометрическими константами: радиусом колеса и колеёй ``kTrackWidthM``.

Ошибка в радиусе даёт масштабную ошибку всего пути: робот думает, что проехал
метр, а проехал девяносто сантиметров. Ошибка в колее даёт систематический
недоворот или переворот. Оба дефекта на графике скорости выглядят идеально
и проявляются только на замкнутом маршруте.

Сценарии
--------

``triangle``
    Вперёд, поворот на 90 градусов, вперёд, возврат по гипотенузе.
    Замкнутый маршрут: ошибка накапливается и видна как невязка в точке старта.

``rectangle``
    Прямоугольник: четыре стороны и четыре поворота по 90 градусов.
    Больше поворотов, поэтому чувствительнее к ошибке колеи.

``figure8``
    Восьмёрка: две окружности в разные стороны. Повороты в обе стороны
    компенсируют друг друга, поэтому несимметричная ошибка колеи здесь
    видна отчётливее, чем на маршруте с поворотами в одну сторону.

Робот должен стоять НА ПОЛУ и иметь свободное место вокруг. Это единственный
инструмент набора, который требует именно поездки, а не поднятого робота.

Управление разомкнутое: отрезки задаются временем при известной скорости.
Поэтому тест проверяет одометрию, а не точность следования маршруту.

Примеры::

    python3 route_test.py --port /dev/ttyUSB0 --pattern triangle
    python3 route_test.py --port /dev/ttyUSB0 --pattern figure8 --speed 0.25
    python3 route_test.py --port /dev/ttyUSB0 --pattern rectangle --sweep
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# protocol/ - общий кодек, использует и pi/, и pc/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "protocol"))

from bench import BenchLink, Sample, TelemetryLogger  # noqa: E402
from plotting import (  # noqa: E402
    TrajectoryWindow,
    require_plotting,
    set_theme,
    theme_choices,
)
from rtk_link import pack_velocity_command  # noqa: E402

COMMAND_PERIOD_S = 0.02

#: Пауза между отрезками. Нужна, чтобы разгон следующего отрезка не начинался
#: с остатка предыдущего: иначе поворот смазывается в дугу.
SEGMENT_PAUSE_S = 0.6



@dataclass
class Segment:
    """Один отрезок маршрута: команда корпуса и её длительность."""

    label: str
    linear_mps: float
    angular_rps: float
    duration_s: float




@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0




@dataclass
class RouteResult:
    """Итог одного прогона маршрута."""

    pattern: str
    speed_mps: float
    ideal_x: list[float] = field(default_factory=list)
    ideal_y: list[float] = field(default_factory=list)
    ideal_heading: list[float] = field(default_factory=list)
    actual_x: list[float] = field(default_factory=list)
    actual_y: list[float] = field(default_factory=list)
    actual_heading: list[float] = field(default_factory=list)
    times: list[float] = field(default_factory=list)

    @property
    def closure_error_m(self) -> float:
        """Расстояние от финиша до старта по мнению одометрии.

        Для замкнутого маршрута идеал равен нулю. Ненулевая невязка означает
        либо ошибку геометрии, либо проскальзывание колёс.
        """

        if not self.actual_x:
            return 0.0
        return math.hypot(self.actual_x[-1], self.actual_y[-1])

    @property
    def final_position_error_m(self) -> float:
        """Расхождение конечных точек идеального и энкодерного путей."""

        if not self.actual_x or not self.ideal_x:
            return 0.0
        return math.hypot(
            self.actual_x[-1] - self.ideal_x[-1],
            self.actual_y[-1] - self.ideal_y[-1],
        )

    @property
    def final_heading_error_rad(self) -> float:
        if not self.actual_heading or not self.ideal_heading:
            return 0.0
        return wrap_angle(self.actual_heading[-1] - self.ideal_heading[-1])

    @property
    def ideal_length_m(self) -> float:
        return path_length(self.ideal_x, self.ideal_y)

    @property
    def actual_length_m(self) -> float:
        return path_length(self.actual_x, self.actual_y)

    @property
    def scale_ratio(self) -> float:
        """Отношение пройденного пути к желаемому.

        Отклонение от единицы указывает прямо на радиус колеса: одометрия
        масштабируется им линейно.
        """

        ideal = self.ideal_length_m
        return self.actual_length_m / ideal if ideal > 1e-6 else 0.0




def wrap_angle(angle: float) -> float:
    """Нормализовать угол в [-pi, pi]."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi




def path_length(xs: list[float], ys: list[float]) -> float:
    return sum(
        math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs))
    )




def build_triangle(speed: float, turn_rate: float, side: float) -> list[Segment]:
    """Вперёд, поворот на 90 градусов, вперёд, возврат по гипотенузе."""

    quarter_turn = (math.pi / 2.0) / turn_rate
    hypotenuse = side * math.sqrt(2.0)

    # Развернуться на 135 градусов, чтобы встать на гипотенузу.
    return [
        Segment("сторона A", speed, 0.0, side / speed),
        Segment("поворот 90", 0.0, turn_rate, quarter_turn),
        Segment("сторона B", speed, 0.0, side / speed),
        Segment("разворот 135", 0.0, turn_rate, (3.0 * math.pi / 4.0) / turn_rate),
        Segment("гипотенуза", speed, 0.0, hypotenuse / speed),
    ]




def build_rectangle(speed: float, turn_rate: float, side: float) -> list[Segment]:
    """Прямоугольник со сторонами side и side/2."""

    quarter_turn = (math.pi / 2.0) / turn_rate
    segments: list[Segment] = []

    for index, length in enumerate((side, side / 2.0, side, side / 2.0)):
        segments.append(Segment(f"сторона {index + 1}", speed, 0.0, length / speed))
        segments.append(Segment(f"поворот {index + 1}", 0.0, turn_rate, quarter_turn))

    return segments




def build_figure8(speed: float, turn_rate: float, side: float) -> list[Segment]:
    """Две полные окружности в разные стороны."""

    full_circle = (2.0 * math.pi) / turn_rate

    return [
        Segment("круг влево", speed, turn_rate, full_circle),
        Segment("круг вправо", speed, -turn_rate, full_circle),
    ]



PATTERNS = {
    "triangle": build_triangle,
    "rectangle": build_rectangle,
    "figure8": build_figure8,
}



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Проверить одометрию на замкнутом маршруте"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Скорость порта")
    parser.add_argument(
        "--pattern",
        default="triangle",
        choices=sorted(PATTERNS),
        help="Сценарий маршрута (по умолчанию triangle)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.20,
        help="Линейная скорость, м/с (по умолчанию 0.20)",
    )
    parser.add_argument(
        "--turn-rate",
        type=float,
        default=0.60,
        help="Угловая скорость на поворотах, рад/с (по умолчанию 0.60)",
    )
    parser.add_argument(
        "--side",
        type=float,
        default=1.0,
        help="Характерный размер маршрута, м (по умолчанию 1.0)",
    )
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="Прогнать маршрут на нескольких скоростях подряд",
    )
    parser.add_argument(
        "--sweep-speeds",
        default="0.10,0.20,0.30",
        help="Скорости для --sweep через запятую",
    )
    parser.add_argument("--log", type=Path, help="Записать телеметрию в CSV")
    parser.add_argument(
        "--no-plot", action="store_true", help="Не открывать окно траектории"
    )
    parser.add_argument(
        "--theme",
        default="light",
        choices=theme_choices(),
        help="Тема графиков (по умолчанию светлая)",
    )
    return parser.parse_args()




def integrate_ideal(segments: list[Segment], dt: float = 0.02) -> tuple[
    list[float], list[float], list[float], list[float]
]:
    """Проинтегрировать команду, получив путь идеально исполняющего робота.

    Это не «истина», а желаемое: то, куда робот приехал бы, если бы точно
    держал команду и геометрия была верна. Расхождение с энкодерным путём
    объединяет ошибки регулятора, геометрии и проскальзывания.
    """

    pose = Pose()
    xs, ys, headings, times = [pose.x], [pose.y], [pose.heading], [0.0]
    elapsed = 0.0

    for segment in segments:
        steps = max(1, int(round(segment.duration_s / dt)))
        step_dt = segment.duration_s / steps

        for _ in range(steps):
            delta_heading = segment.angular_rps * step_dt
            heading_mid = pose.heading + 0.5 * delta_heading
            distance = segment.linear_mps * step_dt

            pose.x += distance * math.cos(heading_mid)
            pose.y += distance * math.sin(heading_mid)
            pose.heading = wrap_angle(pose.heading + delta_heading)
            elapsed += step_dt

            xs.append(pose.x)
            ys.append(pose.y)
            headings.append(pose.heading)
            times.append(elapsed)

        # Пауза между отрезками: команда нулевая, поза не меняется.
        elapsed += SEGMENT_PAUSE_S
        xs.append(pose.x)
        ys.append(pose.y)
        headings.append(pose.heading)
        times.append(elapsed)

    return xs, ys, headings, times




def run_route(
    link: BenchLink,
    segments: list[Segment],
    pattern: str,
    speed: float,
    logger: TelemetryLogger | None,
) -> RouteResult:
    """Проехать маршрут и собрать энкодерную траекторию."""

    result = RouteResult(pattern=pattern, speed_mps=speed)

    link.stop(0.5)
    link.reset(odometry=True, pid=True)
    time.sleep(0.3)

    collected: list[Sample] = []
    start_time = None

    for segment in segments:
        print(
            f"  segment={segment.label:<14} linear_mps={segment.linear_mps:+.3f} "
            f"angular_rps={segment.angular_rps:+.3f} "
            f"duration_s={segment.duration_s:.2f}"
        )

        if logger is not None:
            logger.set_phase(f"{pattern}_{speed:.2f}_{segment.label}")

        command = pack_velocity_command(segment.linear_mps, segment.angular_rps)
        samples = link.hold(command, segment.duration_s)

        if logger is not None:
            logger.write_all(samples)
        collected.extend(samples)

        # Пауза, чтобы отрезки не смазывались друг в друга.
        if logger is not None:
            logger.set_phase(f"{pattern}_{speed:.2f}_пауза")
        pause = link.stop(SEGMENT_PAUSE_S)
        if logger is not None:
            logger.write_all(pause)
        collected.extend(pause)

    if not collected:
        raise SystemExit("Телеметрия за время маршрута не пришла.")

    start_time = collected[0].time_s
    for sample in collected:
        result.actual_x.append(sample.telemetry.odom_x_m)
        result.actual_y.append(sample.telemetry.odom_y_m)
        result.actual_heading.append(sample.telemetry.odom_heading_rad)
        result.times.append(sample.time_s - start_time)

    ideal_x, ideal_y, ideal_heading, ideal_times = integrate_ideal(segments)
    result.ideal_x = ideal_x
    result.ideal_y = ideal_y
    result.ideal_heading = ideal_heading

    return result




def summarise(result: RouteResult) -> str:
    """Свести прогон к измеренным величинам.

    Только числа. Толкование - что именно означает расхождение масштаба или
    курса - остаётся за тем, кто проводит испытание: одни и те же цифры
    на разном покрытии и при разной загрузке значат разное.
    """

    lines = [
        f"pattern = {result.pattern}",
        f"speed_mps = {result.speed_mps:.3f}",
        f"samples = {len(result.times)}",
        "",
        # Все длины в метрах, углы в градусах. Идеальный путь получен
        # интегрированием поданной команды, фактический - одометрией прошивки.
        f"  closure_error_m        = {result.closure_error_m:.4f}",
        f"  final_position_error_m = {result.final_position_error_m:.4f}",
        f"  final_heading_error_deg = "
        f"{math.degrees(result.final_heading_error_rad):+.2f}",
        f"  ideal_path_length_m    = {result.ideal_length_m:.4f}",
        f"  actual_path_length_m   = {result.actual_length_m:.4f}",
        f"  path_length_ratio      = {result.scale_ratio:.4f}",
    ]

    # Невязка в долях длины маршрута: абсолютное значение несравнимо
    # между маршрутами разной длины.
    if result.ideal_length_m > 1e-6:
        lines.append(
            f"  closure_error_ratio    = "
            f"{result.closure_error_m / result.ideal_length_m:.4f}"
        )

    return "\n".join(lines)




def main() -> int:
    args = parse_args()
    set_theme(args.theme)

    speeds = [args.speed]
    if args.sweep:
        speeds = [float(value) for value in args.sweep_speeds.split(",")]

    print(f"route_test: pattern={args.pattern} turn_rate_rps={args.turn_rate:.3f} "
          f"side_m={args.side:.3f}")
    print("робот должен стоять на полу, вокруг нужно свободное место")
    input("Enter для старта... ")

    logger = TelemetryLogger(args.log) if args.log else None
    window = None
    results: list[RouteResult] = []

    try:
        with BenchLink(args.port, args.baud) as link:
            for speed in speeds:
                builder = PATTERNS[args.pattern]
                segments = builder(speed, args.turn_rate, args.side)

                print(f"\nrun speed_mps={speed:.3f}")
                result = run_route(link, segments, args.pattern, speed, logger)
                results.append(result)

                print()
                print(summarise(result))

                if len(speeds) > 1 and speed != speeds[-1]:
                    print("\nпауза")
                    link.stop(2.0)
    except KeyboardInterrupt:
        print("\nПрервано пользователем.")
        return 130
    finally:
        if logger is not None:
            logger.close()
            print(f"log = {logger.path} ({logger.rows} rows)")

    if not args.no_plot and results and require_plotting():
        # Показываем последний прогон: при развёртке по скоростям остальные
        # остаются в логе и в текстовом отчёте.
        result = results[-1]
        window = TrajectoryWindow(f"Маршрут {result.pattern}")

        summary = summarise(result)
        if len(results) > 1:
            summary += "\n\nother_runs:\n"
            for other in results[:-1]:
                summary += (
                    f"  speed_mps={other.speed_mps:.3f} "
                    f"closure_error_m={other.closure_error_m:.4f} "
                    f"path_length_ratio={other.scale_ratio:.4f}\n"
                )

        window.update(
            (result.ideal_x, result.ideal_y),
            (result.actual_x, result.actual_y),
            result.times,
            _resample(result.ideal_heading, len(result.times)),
            result.actual_heading,
            summary,
        )
        print("\nокно открыто, закройте для выхода")
        window.show_blocking()

    return 0



def _resample(values: list[float], length: int) -> list[float]:
    """Растянуть ряд на нужное число точек.

    Идеальный путь считается с собственным шагом, а телеметрия приходит
    со своим, поэтому для общего графика курса ряды приводятся к одной длине.
    """

    if not values or length <= 0:
        return [0.0] * max(0, length)

    if len(values) == length:
        return list(values)

    scale = (len(values) - 1) / max(1, length - 1)
    return [values[min(len(values) - 1, int(round(index * scale)))]
            for index in range(length)]


if __name__ == "__main__":
    raise SystemExit(main())
