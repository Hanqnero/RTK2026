#!/usr/bin/env python3
"""Автоматический подбор ПИД одного колеса релейным методом.

Считает не этот скрипт: подбором занимается ``PIDtuner`` из GyverPID,
который живёт в прошивке. Иначе нельзя - тюнер обязан работать в темпе
управляющего цикла, а через сеть такой темп не выдержать: раскачка идёт
на 25 мс, любая сетевая задержка исказила бы период автоколебаний, из
которого и выводятся коэффициенты.

Как работает релейный метод
---------------------------

Тюнер выводит колесо на ``--steady-pwm``, дожидается установившейся
скорости, затем начинает переключать PWM на ``--step-pwm`` в обе стороны
каждый раз, когда скорость пересекает установившуюся. Система входит
в автоколебания, а по их периоду и амплитуде считаются предельный
коэффициент усиления и период, из которых и получаются kp, ki и kd.

Что он НЕ измеряет
------------------

Feedforward. ``k_static`` и ``k_velocity`` снимаются identify_wheel.py и
переживают автотюн: прошивка меняет только три коэффициента регулятора.
Поэтому порядок остаётся прежним - сначала identify_wheel.py, потом этот
скрипт.

Робота надо поднять над поверхностью: во время подбора колесо раскачивается
на полном размахе ступеньки, а регуляторы отключены.

Примеры::

    python3 autotune_wheel.py --port $RTK_LINK --wheel right
    python3 autotune_wheel.py --port $RTK_LINK --wheel right --save
    python3 autotune_wheel.py --port $RTK_LINK --wheel left \
        --steady-pwm 110 --step-pwm 40
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

from bench import (  # noqa: E402
    WHEEL_NAMES,
    BenchLink,
    TelemetryLogger,
    wheel_index,
)
from plotting import (  # noqa: E402
    LiveMonitorWindow,
    require_plotting,
    set_theme,
    theme_choices,
)
from tune_wheel import export_yaml, print_gains  # noqa: E402

from rtk_link import AutotuneStatus, WheelGains  # noqa: E402

#: Базовый PWM по умолчанию.
#:
#: Обязан быть заметно выше PWM страгивания, иначе колесо стоит в мёртвой
#: зоне: раскачки не возникает, и тюнер зависает на этапе стабилизации.
#: Значение снимается identify_wheel.py как deadband_pwm.
DEFAULT_STEADY_PWM = 90

#: Амплитуда релейной ступеньки.
#:
#: Слишком малая не выводит систему из мёртвой зоны трения, слишком большая
#: загоняет мотор в насыщение, и период автоколебаний перестаёт отражать
#: динамику контура.
DEFAULT_STEP_PWM = 30

#: Сколько пакетов держать в окне графика. При 40 Гц телеметрии это около
#: двадцати секунд - достаточно, чтобы видеть несколько периодов раскачки.
PLOT_CAPACITY = 800

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
        description="Подобрать ПИД колеса автотюнером GyverPID"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Скорость порта")
    parser.add_argument(
        "--wheel",
        default="right",
        choices=("left", "right"),
        help="Настраиваемое колесо",
    )
    parser.add_argument(
        "--steady-pwm",
        type=int,
        default=DEFAULT_STEADY_PWM,
        help=f"Базовый PWM раскачки (по умолчанию {DEFAULT_STEADY_PWM})",
    )
    parser.add_argument(
        "--step-pwm",
        type=int,
        default=DEFAULT_STEP_PWM,
        help=f"Амплитуда ступеньки (по умолчанию {DEFAULT_STEP_PWM})",
    )
    parser.add_argument(
        "--wait",
        type=int,
        default=1500,
        help="Ожидание установившейся скорости перед раскачкой, мс",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=0.05,
        help="Порог скорости изменения для признания системы устоявшейся, об/с",
    )
    parser.add_argument(
        "--pulse",
        type=int,
        default=400,
        help="Длительность первого импульса раскачки, мс",
    )
    parser.add_argument(
        "--accuracy",
        type=int,
        default=90,
        help="Совпадение соседних периодов, при котором подбор завершается, %%",
    )
    parser.add_argument(
        "--tuner-period",
        type=int,
        default=100,
        help=(
            "Период итерации тюнера, мс. Медленнее управляющего цикла "
            "намеренно: на 25 мс реле переключается раньше, чем скорость "
            "уйдёт от средней линии, и коэффициенты выходят завышенными "
            "в десятки раз (по умолчанию 100)"
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Сколько ждать завершения подбора, секунды",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Записать найденные коэффициенты в EEPROM",
    )
    parser.add_argument("--export", type=Path, help="Записать коэффициенты в YAML")
    parser.add_argument("--log", type=Path, help="Записать телеметрию в CSV")
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Открыть окно с раскачкой: скорости колёс и подаваемый PWM",
    )
    parser.add_argument(
        "--theme",
        default="light",
        choices=theme_choices(),
        help="Тема графиков (по умолчанию светлая)",
    )
    return parser.parse_args()




def show_status(status: AutotuneStatus, started: float) -> None:
    """Одна строка на каждый отчёт прошивки о ходе подбора."""

    elapsed = time.monotonic() - started

    # Коэффициенты становятся осмысленными только на этапе автоколебаний:
    # до него тюнер их не считал, и печатать нули было бы враньём.
    if status.state >= 3:
        gains = (
            f"kp={status.kp:7.3f} ki={status.ki:7.3f} kd={status.kd:6.3f}"
        )
    else:
        gains = "коэффициенты ещё не считаются"

    print(
        f"{elapsed:6.1f} с  {status.stage_name:<22} "
        f"точность={status.accuracy:3d} %  {gains}",
        flush=True,
    )




class PlotFeed:
    """Накопитель серий для живого окна.

    Во время автотюна уставки нулевые: PWM держит сам тюнер в обход
    регуляторов. Кривые уставок остаются на нуле намеренно - врать о цели,
    которой никто не добивается, панель не должна.
    """

    def __init__(self, window: LiveMonitorWindow, started: float) -> None:
        self._window = window
        self._started = started
        self._series = {
            key: deque(maxlen=PLOT_CAPACITY) for key in SERIES_KEYS
        }
        self._last_redraw = 0.0
        self._status_line = "ожидание телеметрии"


    def set_status(self, text: str) -> None:
        self._status_line = text


    def add(self, sample) -> None:
        self._series["time"].append(sample.time_s - self._started)

        for index, prefix in ((0, "left"), (1, "right")):
            setpoint = sample.setpoint_rps(index)
            measured = sample.wheel_rps(index)
            self._series[f"{prefix}_setpoint"].append(setpoint)
            self._series[f"{prefix}_measured"].append(measured)
            self._series[f"{prefix}_pwm"].append(float(sample.pwm(index)))
            self._series[f"{prefix}_error"].append(setpoint - measured)

        # Перерисовка реже прихода пакетов: иначе Qt съедает время,
        # которое нужно на повтор команды.
        now = time.monotonic()
        if now - self._last_redraw >= 0.05:
            self._last_redraw = now
            self.redraw()


    def redraw(self) -> None:
        self._window.update(
            {key: list(values) for key, values in self._series.items()},
            self._status_line,
        )


    @property
    def closed(self) -> bool:
        return self._window.closed




def status_text(status: AutotuneStatus, started: float) -> str:
    """Многострочная сводка для текстового блока панели."""

    lines = [
        f"колесо: {status.wheel_name}",
        f"этап: {status.stage_name}",
        f"точность автоколебаний: {status.accuracy} %",
        f"прошло: {time.monotonic() - started:.1f} с",
    ]

    if status.state >= 3:
        lines.append(
            f"kp={status.kp:.4f}  ki={status.ki:.4f}  kd={status.kd:.4f}"
        )
    else:
        lines.append("коэффициенты появятся на этапе автоколебаний")

    if status.is_done:
        lines.append("ПОДБОР ЗАВЕРШЁН")

    return "\n".join(lines)




def main() -> int:
    args = parse_args()
    wheel = wheel_index(args.wheel)
    set_theme(args.theme)

    if args.step_pwm <= 0:
        raise SystemExit("--step-pwm должен быть положительным: нечем раскачивать")

    print("autotune: робот должен быть поднят над поверхностью")
    print(
        f"колесо = {WHEEL_NAMES[wheel]}, steady_pwm = {args.steady_pwm}, "
        f"step_pwm = {args.step_pwm}, целевая точность = {args.accuracy} %"
    )
    time.sleep(1.0)

    logger = TelemetryLogger(args.log) if args.log else None
    started = time.monotonic()

    try:
        with BenchLink(args.port, args.baud) as link:
            link.reset(odometry=True, pid=True, stats=True)

            print()
            print("до подбора:")
            print_gains(link)
            print()

            if logger is not None:
                logger.set_phase(f"autotune_{WHEEL_NAMES[wheel]}")

            feed = None
            if args.plot and require_plotting():
                feed = PlotFeed(
                    LiveMonitorWindow(
                        window_s=20.0,
                        title=f"Автотюн ПИД: {WHEEL_NAMES[wheel]}",
                    ),
                    started,
                )

            def on_sample(sample) -> None:
                if logger is not None:
                    logger.write(sample)
                if feed is not None:
                    feed.add(sample)

            def on_status(status: AutotuneStatus) -> None:
                show_status(status, started)
                if feed is not None:
                    feed.set_status(status_text(status, started))

            result = link.run_autotune(
                wheel=wheel,
                steady_pwm=args.steady_pwm,
                step_pwm=args.step_pwm,
                wait_ms=args.wait,
                window_rps=args.window,
                pulse_ms=args.pulse,
                target_accuracy=args.accuracy,
                tuner_period_ms=args.tuner_period,
                timeout_s=args.timeout,
                on_status=on_status,
                on_sample=on_sample,
            )

            print()
            print(
                f"подбор завершён за {time.monotonic() - started:.1f} с, "
                f"точность автоколебаний {result.accuracy} %"
            )

            # Прошивка уже перенесла найденные коэффициенты в регулятор,
            # но живут они пока только в оперативной памяти.
            if args.save:
                link.save_gains()
                print("eeprom_write = ok")
            else:
                print("eeprom_write = no (--save не задан)")

            print()
            print("после подбора:")
            print_gains(link)

            if args.export:
                gains: dict[int, WheelGains] = {
                    index: report.gains
                    for index, report in link.get_gains().items()
                }
                export_yaml(args.export, gains)
                print(f"export = {args.export}")

            if result.accuracy < args.accuracy:
                print()
                print(
                    "ВНИМАНИЕ: точность ниже запрошенной. Коэффициенты выведены "
                    "из неустойчивой раскачки и достоверны лишь отчасти."
                )

            if feed is not None:
                print()
                print("график раскачки открыт, закройте окно для выхода")
                feed.set_status(status_text(result, started))
                while not feed.closed:
                    feed.redraw()
                    time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nПрервано пользователем.")
        return 130
    finally:
        if logger is not None:
            logger.close()
            print(f"log = {logger.path} ({logger.rows} rows)")

    return 0



if __name__ == "__main__":
    raise SystemExit(main())
