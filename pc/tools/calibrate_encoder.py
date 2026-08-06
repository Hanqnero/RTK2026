#!/usr/bin/env python3
"""Измерение числа отсчётов энкодера на оборот колеса.

Это первый шаг настройки привода, и пропустить его нельзя: без верного числа
отсчётов неправильны и одометрия, и скорости, и единицы feedforward, поэтому
всё, что настроено дальше, настроено под неверную шкалу.

Почему величину нельзя вывести из паспорта
------------------------------------------

Отсчётов на оборот колеса = импульсов на канал x 4 x передаточное отношение.

Энкодер JGB37-520 стоит на валу мотора, ДО редуктора, поэтому редуктор входит
в пересчёт обязательно. При этом паспорт даёт обороты выходного вала, а не
отношение, число импульсов бывает 11 или 13, а сами отношения нередко
нецелые. Итог измеряется, а не вычисляется.

Два режима
----------

``--manual`` - колесо проворачивается рукой на целое число оборотов.
Точнее всего: нет ни проскальзывания, ни нагрузки. Мотор при этом обесточен.

``--powered`` - колесо крутится само на заданной скорости заданное время.
Быстрее и не требует рук, но опирается на паспортные обороты, поэтому годится
для проверки, а не для первичного измерения.

Робота нужно поднять над поверхностью в обоих режимах.

Примеры::

    python3 calibrate_encoder.py --port /dev/ttyUSB0 --manual --turns 10
    python3 calibrate_encoder.py --port /dev/ttyUSB0 --powered --rps 2.0
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

from transport import open_transport  # noqa: E402

from rtk_link import (  # noqa: E402
    MSG_TELEMETRY,
    RESET_ODOMETRY,
    RESET_PID,
    FrameDecoder,
    SequenceTracker,
    decode_telemetry,
    pack_reset_command,
    pack_velocity_command,
    pack_wheel_setpoint_command,
)

COMMAND_PERIOD_S = 0.02

# Типовые варианты JGB37-520: обороты выходного вала и соответствующее
# передаточное отношение. Нужны только для подсказки в отчёте.
KNOWN_VARIANTS = [
    (960.0, 5.2),
    (600.0, 9.0),
    (320.0, 19.0),
    (200.0, 30.0),
    (107.0, 56.0),
    (66.0, 90.0),
]



@dataclass
class Sample:
    left_total: int
    right_total: int
    left_rps: float
    right_rps: float




def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Измерить число отсчётов энкодера на оборот колеса"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Скорость порта")

    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--manual",
        action="store_true",
        help="Проворачивать колесо рукой (по умолчанию, самый точный режим)",
    )
    mode.add_argument(
        "--powered",
        action="store_true",
        help="Крутить колесо мотором по паспортной скорости",
    )

    parser.add_argument(
        "--turns",
        type=float,
        default=10.0,
        help="Сколько оборотов проворачивается рукой (по умолчанию 10)",
    )
    parser.add_argument(
        "--rps",
        type=float,
        default=2.0,
        help="Скорость колеса в режиме --powered, оборотов в секунду",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Длительность вращения в режиме --powered, секунды",
    )
    parser.add_argument(
        "--pulses-per-rev",
        type=float,
        default=11.0,
        help="Импульсов на канал за оборот вала мотора (по умолчанию 11)",
    )
    return parser.parse_args()




def read_latest(port, decoder: FrameDecoder, sequence: SequenceTracker) -> Sample | None:
    """Забрать самый свежий пакет телеметрии из буфера."""

    latest = None
    waiting = port.in_waiting

    if not waiting:
        return None

    for message_id, payload in decoder.feed(port.read(waiting)):
        if message_id != MSG_TELEMETRY:
            continue

        telemetry = decode_telemetry(payload)
        sequence.update(telemetry.seq)
        latest = Sample(
            left_total=telemetry.left_encoder_total,
            right_total=telemetry.right_encoder_total,
            left_rps=telemetry.left_wheel_rps,
            right_rps=telemetry.right_wheel_rps,
        )

    return latest




def wait_for_sample(port, decoder, sequence, timeout_s: float = 5.0) -> Sample:
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        sample = read_latest(port, decoder, sequence)
        if sample is not None:
            return sample
        time.sleep(0.005)

    raise SystemExit(
        "Телеметрия не поступает. Проверьте порт и что прошита версия протокола 2."
    )




def hold_command(port, decoder, sequence, command: bytes, seconds: float) -> Sample:
    """Удерживать команду заданное время и вернуть последний пакет."""

    latest = None
    deadline = time.monotonic() + seconds
    next_send = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()

        if now >= next_send:
            port.write(command)
            next_send = now + COMMAND_PERIOD_S

        sample = read_latest(port, decoder, sequence)
        if sample is not None:
            latest = sample

        time.sleep(0.002)

    return latest if latest is not None else wait_for_sample(port, decoder, sequence)




def report(counts_per_rev: float, wheel: str, pulses_per_rev: float) -> None:
    """Показать измерение и ближайший паспортный вариант мотора."""

    gear = counts_per_rev / (pulses_per_rev * 4.0)

    print(f"  {wheel:5s}: {counts_per_rev:8.1f} отсчётов на оборот колеса")
    print(f"         передаточное отношение ≈ 1:{gear:.2f}")

    closest = min(KNOWN_VARIANTS, key=lambda item: abs(item[1] - gear))
    rpm, known_gear = closest
    error_share = abs(known_gear - gear) / known_gear

    if error_share < 0.10:
        print(
            f"         ближайший вариант: {rpm:.0f} об/мин (1:{known_gear:.1f}), "
            f"расхождение {100.0 * error_share:.1f} %"
        )
    else:
        print(
            f"         ни один типовой вариант не подходит "
            f"(ближайший 1:{known_gear:.1f}). Проверьте --pulses-per-rev: "
            "у части энкодеров 13 импульсов, а не 11."
        )




def main() -> int:
    args = parse_args()
    powered = args.powered

    port = open_transport(args.port, args.baud)

    decoder = FrameDecoder()
    sequence = SequenceTracker()

    print(f"Порт {args.port} @ {args.baud}, протокол v2")
    print("Поднимите робота над поверхностью.")
    time.sleep(2.0)
    port.reset_input_buffer()
    port.write(pack_reset_command(RESET_ODOMETRY | RESET_PID))

    try:
        if powered:
            print(
                f"\nРежим --powered: {args.rps:.2f} об/с в течение "
                f"{args.duration:.1f} с.\n"
                "ВНИМАНИЕ: скорость задаётся по текущим константам прошивки, "
                "которые мы и проверяем.\n"
                "Режим годится для контроля, но не для первичного измерения."
            )

            start = wait_for_sample(port, decoder, sequence)
            command = pack_wheel_setpoint_command(args.rps, args.rps)
            end = hold_command(port, decoder, sequence, command, args.duration)

            # Останавливаемся до расчётов, чтобы колёса не крутились впустую.
            port.write(pack_velocity_command(0.0, 0.0))

            expected_turns = args.rps * args.duration
            print(f"\nОжидалось оборотов по уставке: {expected_turns:.2f}")
        else:
            print(
                f"\nРежим --manual: проверните КАЖДОЕ колесо ровно на "
                f"{args.turns:.0f} оборотов вперёд."
            )
            print("Отметьте начальное положение колеса, чтобы не сбиться.")
            input("Нажмите Enter, когда будете готовы начать... ")

            start = wait_for_sample(port, decoder, sequence)
            print("Считаю. Проворачивайте колёса.")
            input("Нажмите Enter, когда закончите... ")

            # Дренируем буфер, чтобы взять действительно последний пакет.
            time.sleep(0.2)
            end = wait_for_sample(port, decoder, sequence)
            expected_turns = args.turns

        left_counts = end.left_total - start.left_total
        right_counts = end.right_total - start.right_total

        print(f"\nОтсчётов набрано: левое {left_counts:+d}, правое {right_counts:+d}")

        if sequence.lost:
            print(
                f"Потеряно пакетов: {sequence.lost}. На результат это не влияет: "
                "используются накопленные счётчики, а не сумма дельт."
            )

        if expected_turns <= 0:
            print("Нулевое число оборотов, считать нечего.", file=sys.stderr)
            return 2

        if left_counts == 0 and right_counts == 0:
            print(
                "\nНи один энкодер не дал отсчётов. Проверьте питание энкодеров "
                "(3.3 В), подключение C1/C2 и распиновку в motor_interface.h.",
                file=sys.stderr,
            )
            return 2

        print("\nРезультат:")
        for wheel, counts in (("левое", left_counts), ("правое", right_counts)):
            if counts == 0:
                print(f"  {wheel:5s}: отсчётов нет, канал не читается")
                continue
            report(abs(counts) / expected_turns, wheel, args.pulses_per_rev)

        if left_counts and right_counts:
            asymmetry = abs(abs(left_counts) - abs(right_counts)) / max(
                abs(left_counts), abs(right_counts)
            )
            print(f"\nРасхождение между колёсами: {100.0 * asymmetry:.2f} %")
            if asymmetry > 0.02:
                print(
                    "  Больше двух процентов. Либо колёса провернули на разное "
                    "число оборотов, либо у моторов разные редукторы."
                )

        if left_counts < 0 or right_counts < 0:
            print(
                "\nОтрицательные отсчёты при вращении вперёд означают неверный "
                "флаг k*EncoderReverse в motor_interface.h. Проверьте "
                "verify_signs.py."
            )

        average = (abs(left_counts) + abs(right_counts)) / 2.0 / expected_turns
        print(
            f"\nПодставьте в motor_interface.h передаточное отношение так, чтобы\n"
            f"kEncoderCountsPerWheelRev получилось ≈ {average:.0f}:\n"
            f"  kGearRatio = {average / (args.pulses_per_rev * 4.0):.3f}"
        )
        return 0
    except KeyboardInterrupt:
        print("\nПрервано пользователем.")
        return 130
    finally:
        try:
            port.write(pack_velocity_command(0.0, 0.0))
            time.sleep(0.1)
        except OSError:
            pass
        port.close()



if __name__ == "__main__":
    raise SystemExit(main())
