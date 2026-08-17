#!/usr/bin/env python3
"""Настройка ПИД одного колеса по ступенчатому отклику.

Третий шаг настройки, после calibrate_encoder.py и identify_wheel.py.

Порядок обязателен: без верного числа отсчётов скорости выражены в неверных
единицах, а без готового feedforward коэффициенты ПИД получаются на порядок
больше нужных. Прежние большие коэффициенты поверх точного feedforward дают
заброс порядка 67 процентов - ПИД начинает добавлять к уже верному PWM ещё
столько же.

Инструмент даёт цикл «поменял коэффициент - увидел отклик - сравнил с прошлым»,
не перепрошивая плату. Коэффициенты действуют сразу, но до команды ``save``
живут только в оперативной памяти.

Робота нужно поднять над поверхностью: колёса будут вращаться.

Примеры::

    python3 tune_wheel.py --port /dev/ttyUSB0 --wheel left
    python3 tune_wheel.py --port /dev/ttyUSB0 --wheel right --kp 3 --ki 5 --step 2.0
    python3 tune_wheel.py --port /dev/ttyUSB0 --export gains.yaml
"""

from __future__ import annotations

import argparse
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
    Sample,
    TelemetryLogger,
    wheel_index,
)
from plotting import (  # noqa: E402
    StepResponseWindow,
    require_plotting,
    set_theme,
    theme_choices,
)
from rtk_link import (  # noqa: E402
    FLAG_PWM_SATURATED_LEFT,
    FLAG_PWM_SATURATED_RIGHT,
    WHEEL_LEFT,
    WHEEL_RIGHT,
    WheelGains,
)

#: Коридор, по входу в который отклик считается установившимся.
SETTLING_BAND = 0.05



@dataclass
class StepMetrics:
    """Числовая оценка одного ступенчатого отклика."""

    setpoint_rps: float
    rise_time_s: float | None
    settling_time_s: float | None
    overshoot_share: float
    steady_error_rps: float
    rms_error_rps: float
    saturated_share: float
    feedforward_share: float | None
    samples: list[Sample]
    wheel: int

    @property
    def settled(self) -> bool:
        return self.settling_time_s is not None


    def describe(self) -> str:
        """Метрики отклика с явными определениями и единицами.

        rise_time_ms
            От первого превышения 0.1 уставки до первого достижения 0.9.
        settling_time_ms
            Момент, после которого модуль ошибки уже не покидает коридор
            плюс-минус 5 процентов уставки до конца записи.
        overshoot
            Максимум модуля скорости минус уставка, делённое на уставку.
            Ноль при отсутствии заброса.
        steady_state_error_rps
            Уставка минус среднее модуля скорости по последним 20 процентам
            записи.
        rms_error_rps
            Среднеквадратичная ошибка по всей записи.
        pwm_saturation_ratio
            Доля пакетов с взведённым флагом насыщения PWM своего колеса.
        feedforward_ratio
            Модуль feedforward, делённый на модуль выхода регулятора,
            на установившемся участке. Определено только при включённых
            отладочных кадрах.
        """

        rise = (
            f"{self.rise_time_s * 1000.0:.0f}"
            if self.rise_time_s is not None
            else "n/a"
        )
        settling = (
            f"{self.settling_time_s * 1000.0:.0f}"
            if self.settling_time_s is not None
            else "n/a"
        )

        relative_error = (
            abs(self.steady_error_rps) / max(abs(self.setpoint_rps), 1e-9)
        )

        lines = [
            f"  setpoint_rps           = {self.setpoint_rps:.3f}",
            f"  samples                = {len(self.samples)}",
            f"  rise_time_ms           = {rise}",
            f"  settling_time_ms       = {settling}",
            f"  overshoot              = {self.overshoot_share:.4f}",
            f"  steady_state_error_rps = {self.steady_error_rps:+.4f} "
            f"({relative_error:.4f} rel)",
            f"  rms_error_rps          = {self.rms_error_rps:.4f}",
            f"  pwm_saturation_ratio   = {self.saturated_share:.4f}",
        ]

        if self.feedforward_share is not None:
            lines.append(
                f"  feedforward_ratio      = {self.feedforward_share:.4f}"
            )

        return "\n".join(lines)






def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Настроить ПИД одного колеса по ступенчатому отклику"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Скорость порта")
    parser.add_argument(
        "--wheel", default="left", choices=["left", "right"], help="Настраиваемое колесо"
    )
    parser.add_argument("--kp", type=float, help="Начальный Kp")
    parser.add_argument("--ki", type=float, help="Начальный Ki")
    parser.add_argument("--kd", type=float, help="Начальный Kd")
    parser.add_argument(
        "--step",
        type=float,
        help="Выполнить одну ступеньку на заданную скорость и выйти",
    )
    parser.add_argument(
        "--setpoint",
        type=float,
        default=2.0,
        help="Скорость ступеньки по умолчанию, об/с (по умолчанию 2.0)",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=2.0,
        help="Длительность удержания ступеньки, секунды (по умолчанию 2.0)",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Открыть окно с откликом, PWM и составляющими регулятора",
    )
    parser.add_argument(
        "--theme",
        default="light",
        choices=theme_choices(),
        help="Тема графиков (по умолчанию светлая)",
    )
    parser.add_argument(
        "--log",
        type=Path,
        help="Записать телеметрию всех ступенек в CSV",
    )
    parser.add_argument("--export", type=Path, help="Записать коэффициенты в YAML")
    parser.add_argument(
        "--save", action="store_true", help="Записать коэффициенты в EEPROM и выйти"
    )
    return parser.parse_args()




def analyse(samples: list[Sample], wheel: int, setpoint: float) -> StepMetrics:
    """Посчитать метрики ступенчатого отклика."""

    if not samples:
        raise SystemExit("Телеметрия за время ступеньки не пришла.")

    start_time = samples[0].time_s
    speeds = [s.wheel_rps(wheel) for s in samples]
    times = [s.time_s - start_time for s in samples]

    target = abs(setpoint)
    band = SETTLING_BAND * target

    # Нарастание: от 10 до 90 процентов уставки.
    rise_start = rise_end = None
    for moment, speed in zip(times, speeds):
        magnitude = abs(speed)
        if rise_start is None and magnitude >= 0.1 * target:
            rise_start = moment
        if rise_start is not None and magnitude >= 0.9 * target:
            rise_end = moment
            break
    rise_time = (rise_end - rise_start) if (rise_start is not None and rise_end is not None) else None

    # Установление: момент, после которого отклик уже не покидает коридор.
    settling_time = None
    entered_at = None
    for moment, speed in zip(times, speeds):
        if abs(abs(speed) - target) <= band:
            if entered_at is None:
                entered_at = moment
        else:
            entered_at = None
    settling_time = entered_at

    overshoot = max((abs(s) - target for s in speeds), default=0.0)
    overshoot_share = max(0.0, overshoot / target) if target > 1e-9 else 0.0

    tail = max(1, len(speeds) // 5)
    steady_speed = sum(abs(s) for s in speeds[-tail:]) / tail
    steady_error = target - steady_speed

    errors = [target - abs(s) for s in speeds]
    rms_error = (sum(e * e for e in errors) / len(errors)) ** 0.5

    saturation_flag = (
        FLAG_PWM_SATURATED_LEFT if wheel == WHEEL_LEFT else FLAG_PWM_SATURATED_RIGHT
    )
    saturated = sum(1 for s in samples if s.telemetry.flags & saturation_flag)

    # Доля feedforward доступна только если прошивка слала отладочные кадры.
    debug_samples = [s for s in samples[-tail:] if s.debug is not None]
    feedforward_share = None
    if debug_samples:
        shares = []
        for sample in debug_samples:
            fields = sample.debug.left if wheel == WHEEL_LEFT else sample.debug.right
            if abs(fields.output_pwm) > 1e-6:
                shares.append(abs(fields.feedforward) / abs(fields.output_pwm))
        if shares:
            feedforward_share = sum(shares) / len(shares)

    return StepMetrics(
        setpoint_rps=setpoint,
        rise_time_s=rise_time,
        settling_time_s=settling_time,
        overshoot_share=overshoot_share,
        steady_error_rps=steady_error,
        rms_error_rps=rms_error,
        saturated_share=saturated / len(samples),
        feedforward_share=feedforward_share,
        samples=samples,
        wheel=wheel,
    )




def run_step(
    link: BenchLink,
    wheel: int,
    setpoint: float,
    hold_s: float,
    logger: TelemetryLogger | None = None,
) -> StepMetrics:
    """Выполнить ступеньку от нуля до уставки и вернуть метрики."""

    # Начинаем с чистого состояния: остаточный интеграл исказил бы отклик.
    link.stop(0.4)
    link.reset(pid=True)

    left = setpoint if wheel == WHEEL_LEFT else 0.0
    right = setpoint if wheel == WHEEL_RIGHT else 0.0

    if logger is not None:
        # В имя фазы кладём и коэффициенты: иначе при разборе лога
        # невозможно понять, какая ступенька к какой настройке относится.
        gains = link.get_gains()[wheel].gains
        logger.set_phase(
            f"{WHEEL_NAMES[wheel]}_sp{setpoint:.2f}"
            f"_kp{gains.kp:g}_ki{gains.ki:g}_kd{gains.kd:g}"
        )

    samples = link.hold_wheel_setpoint(left, right, hold_s, debug=True)
    link.stop(0.4)

    if logger is not None:
        logger.write_all(samples)
        logger.flush()

    return analyse(samples, wheel, setpoint)




def show_step(metrics: StepMetrics, window: StepResponseWindow | None = None) -> None:
    print()
    print(f"step_response wheel={WHEEL_NAMES[metrics.wheel]}")
    print(metrics.describe())

    if window is not None:
        window.update(*plot_series(metrics), metrics_text=plot_summary(metrics))




def plot_series(metrics: StepMetrics):
    """Разложить ступеньку на ряды для окна графика."""

    start = metrics.samples[0].time_s
    times = [s.time_s - start for s in metrics.samples]

    setpoints = [s.setpoint_rps(metrics.wheel) for s in metrics.samples]
    measured = [s.wheel_rps(metrics.wheel) for s in metrics.samples]
    pwms = [float(s.pwm(metrics.wheel)) for s in metrics.samples]

    # Составляющие доступны только если прошивка слала отладочные кадры.
    terms: dict[str, list[float]] = {}
    if any(s.debug is not None for s in metrics.samples):
        feedforward, proportional, integral = [], [], []

        for sample in metrics.samples:
            if sample.debug is None:
                feedforward.append(0.0)
                proportional.append(0.0)
                integral.append(0.0)
                continue

            fields = (
                sample.debug.left
                if metrics.wheel == WHEEL_LEFT
                else sample.debug.right
            )
            feedforward.append(fields.feedforward)
            proportional.append(fields.proportional)
            integral.append(fields.integral_term)

        terms = {
            "feedforward": feedforward,
            "proportional": proportional,
            "integral": integral,
        }

    return times, setpoints, measured, pwms, terms




def plot_summary(metrics: StepMetrics) -> str:
    """Текстовый блок метрик под графиком."""

    return "\n".join(
        [f"step_response wheel={WHEEL_NAMES[metrics.wheel]}", "", metrics.describe()]
    )




def export_yaml(path: Path, gains: dict[int, WheelGains]) -> None:
    """Сохранить коэффициенты в файл, пригодный для хранения в git.

    EEPROM держит настройку между включениями, но замена платы её теряет,
    поэтому результат стоит держать и в репозитории.
    """

    lines = [
        "# Коэффициенты регуляторов колёс RTK2026.",
        "# Получены tune_wheel.py поверх feedforward из identify_wheel.py.",
        "#",
        "# Единицы: уставка и измерение в оборотах колеса в секунду, выход в PWM.",
        "# Действующие значения живут в EEPROM Arduino; этот файл - копия",
        "# на случай замены платы.",
        f"# Записано: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "wheels:",
    ]

    for wheel in (WHEEL_LEFT, WHEEL_RIGHT):
        value = gains.get(wheel)
        if value is None:
            continue
        lines.append(f"  {WHEEL_NAMES[wheel]}:")
        lines.append(f"    kp: {value.kp:.6f}")
        lines.append(f"    ki: {value.ki:.6f}")
        lines.append(f"    kd: {value.kd:.6f}")
        lines.append(f"    k_static: {value.k_static:.6f}")
        lines.append(f"    k_velocity: {value.k_velocity:.6f}")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")



HELP_TEXT = """
Команды:
  kp <v> / ki <v> / kd <v>    задать коэффициент ПИД
  ks <v> / kv <v>             задать коэффициент feedforward
  step [скорость]             выполнить ступеньку (по умолчанию текущая уставка)
  setpoint <v>                сменить скорость ступеньки по умолчанию
  hold <сек>                  сменить длительность удержания
  wheel left|right            переключить настраиваемое колесо
  show                        показать действующие коэффициенты
  save                        записать в EEPROM
  export <путь>               сохранить коэффициенты в YAML
  defaults                    вернуть скомпилированные значения и стереть EEPROM
  help                        эта справка
  quit                        выйти, остановив приводы
"""



def current_gains(link: BenchLink) -> dict[int, WheelGains]:
    return {wheel: report.gains for wheel, report in link.get_gains().items()}




def print_gains(link: BenchLink) -> None:
    """Действующие коэффициенты обоих колёс.

    Единицы: kp - PWM на оборот в секунду ошибки, ki - то же на секунду,
    kd - то же на секунду в квадрате, k_static - PWM,
    k_velocity - PWM на оборот колеса в секунду.
    """

    print("gains:")
    for wheel, report in sorted(link.get_gains().items()):
        print(
            f"  {report.wheel_name:<5} kp={report.kp:.4f} ki={report.ki:.4f} "
            f"kd={report.kd:.4f} k_static={report.k_static:.3f} "
            f"k_velocity={report.k_velocity:.3f} "
            f"source={'eeprom' if report.is_persisted else 'ram'}"
        )




def interactive(
    link: BenchLink,
    args: argparse.Namespace,
    window: StepResponseWindow | None = None,
    logger: TelemetryLogger | None = None,
) -> int:
    wheel = wheel_index(args.wheel)
    setpoint = args.setpoint
    hold_s = args.hold

    print(HELP_TEXT)
    print_gains(link)
    print(f"wheel = {WHEEL_NAMES[wheel]}")

    while True:
        try:
            raw = input(f"\n[{WHEEL_NAMES[wheel]} sp={setpoint:.2f}] > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0

        if not raw:
            continue

        parts = raw.split()
        command = parts[0].lower()
        argument = parts[1] if len(parts) > 1 else None

        try:
            if command in ("quit", "q", "exit"):
                return 0

            if command in ("help", "?"):
                print(HELP_TEXT)
                continue

            if command == "show":
                print_gains(link)
                continue

            if command == "wheel":
                if argument is None:
                    print("  укажите left или right")
                    continue
                wheel = wheel_index(argument)
                print(f"  wheel = {WHEEL_NAMES[wheel]}")
                continue

            if command == "setpoint":
                setpoint = float(argument)
                continue

            if command == "hold":
                hold_s = float(argument)
                continue

            if command in ("kp", "ki", "kd", "ks", "kv"):
                value = float(argument)
                gains = current_gains(link)[wheel]

                updated = WheelGains(
                    kp=value if command == "kp" else gains.kp,
                    ki=value if command == "ki" else gains.ki,
                    kd=value if command == "kd" else gains.kd,
                    k_static=value if command == "ks" else gains.k_static,
                    k_velocity=value if command == "kv" else gains.k_velocity,
                )

                report = link.set_gains(wheel, updated)
                print(
                    f"  {report.wheel_name:<5} kp={report.kp:.4f} "
                    f"ki={report.ki:.4f} kd={report.kd:.4f} "
                    f"k_static={report.k_static:.3f} "
                    f"k_velocity={report.k_velocity:.3f}"
                )
                continue

            if command == "step":
                target = float(argument) if argument else setpoint
                metrics = run_step(link, wheel, target, hold_s, logger)
                show_step(metrics, window)
                continue

            if command == "save":
                saved = link.save_gains()
                ok = saved and all(r.is_persisted for r in saved.values())
                print(f"  eeprom_write = {'ok' if ok else 'failed'}")
                continue

            if command == "export":
                if argument is None:
                    print("  укажите путь к файлу")
                    continue
                export_yaml(Path(argument), current_gains(link))
                print(f"  export = {argument}")
                continue

            if command == "defaults":
                link.reset(gains_to_default=True)
                time.sleep(0.3)
                print("  gains_reset = compiled, eeprom_cleared = yes")
                print_gains(link)
                continue

            print(f"  неизвестная команда: {command}. Наберите help.")

        except (ValueError, TypeError) as exc:
            print(f"  не понял аргумент: {exc}")
        except SystemExit as exc:
            print(f"  {exc}")




def main() -> int:
    args = parse_args()
    set_theme(args.theme)
    wheel = wheel_index(args.wheel)

    print("tune_wheel: робот должен быть поднят над поверхностью")
    time.sleep(1.0)

    window = None
    if args.plot and require_plotting():
        window = StepResponseWindow(f"Ступенчатый отклик: колесо {args.wheel}")

    logger = TelemetryLogger(args.log) if args.log else None

    # Экспорт выполняется на любом пути выхода: и после одиночной ступеньки,
    # и при записи в EEPROM, и после интерактивной настройки.
    def export_if_requested(link: BenchLink) -> None:
        if not args.export:
            return
        export_yaml(args.export, current_gains(link))
        print(f"Коэффициенты сохранены: {args.export}")

    try:
        with BenchLink(args.port, args.baud) as link:
            reports = link.get_gains()

            # Настраивать ПИД поверх ненастроенного feedforward бессмысленно:
            # коэффициенты уйдут на компенсацию мёртвой зоны, а не на точность.
            target = reports.get(wheel)
            if target is not None and target.k_velocity <= 1e-6:
                print(
                    f"\nВНИМАНИЕ: у колеса {WHEEL_NAMES[wheel]} feedforward "
                    "не измерен (k_velocity = 0).\n"
                    "Сначала запустите identify_wheel.py --apply, иначе ПИД "
                    "будет компенсировать мёртвую зону вместо точности,\n"
                    "и подобранные коэффициенты придётся выбросить после "
                    "идентификации."
                )

            # Начальные коэффициенты из командной строки.
            if any(value is not None for value in (args.kp, args.ki, args.kd)):
                gains = current_gains(link)[wheel]
                link.set_gains(
                    wheel,
                    WheelGains(
                        kp=args.kp if args.kp is not None else gains.kp,
                        ki=args.ki if args.ki is not None else gains.ki,
                        kd=args.kd if args.kd is not None else gains.kd,
                        k_static=gains.k_static,
                        k_velocity=gains.k_velocity,
                    ),
                )

            if args.step is not None:
                show_step(run_step(link, wheel, args.step, args.hold, logger), window)
                if window is not None:
                    print("\nокно открыто, закройте для выхода")
                    window.wait_until_closed()
                export_if_requested(link)
                if args.save:
                    saved = link.save_gains()
                    ok = saved and all(r.is_persisted for r in saved.values())
                    print("Записано в EEPROM" if ok else "ЗАПИСЬ НЕ ПОДТВЕРЖДЕНА")
                return 0

            if args.save:
                export_if_requested(link)
                saved = link.save_gains()
                ok = saved and all(r.is_persisted for r in saved.values())
                print("Записано в EEPROM" if ok else "ЗАПИСЬ НЕ ПОДТВЕРЖДЕНА")
                return 0

            result = interactive(link, args, window, logger)
            export_if_requested(link)

            return result
    except KeyboardInterrupt:
        print("\nПрервано пользователем.")
        return 130
    finally:
        if logger is not None:
            logger.close()
            print(f"log: {logger.path} ({logger.rows} строк)")



if __name__ == "__main__":
    raise SystemExit(main())
