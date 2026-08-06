#!/usr/bin/env python3
"""Проверка согласованности знаков моторов и энкодеров.

Скрипт проверяет контракт, объявленный в ``arduino/include/motor_interface.h``:
положительная команда колеса даёт положительную дельту его энкодера. Это
единственное место, где ориентация железа проверяется экспериментально,
и от неё зависит корректность кинематики, одометрии и телеметрии.

При провале теста правится соответствующий флаг ``k*Reverse`` в
``motor_interface.h`` и ничто другое: компенсирующих минусов в коде
быть не должно.

Робота нужно поднять над поверхностью: колёса будут вращаться.

Здесь колесо задаётся через кинематику корпуса, поэтому проверяется заодно
и правильность ``TRACK_WIDTH_M``. Прямую проверку без пересчёта делает
``check_encoders.py``: он подаёт PWM на колесо напрямую.
"""

from __future__ import annotations

import argparse
import sys
import time
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
    decode_telemetry,
    pack_reset_command,
    pack_velocity_command,
)

# Обязано совпадать с kTrackWidthM из motor_interface.h. Расхождение
# означает, что пересчёт скоростей колёс в уставку корпуса неверен и
# команда «крутить только левое» на деле крутит оба колеса.
TRACK_WIDTH_M = 0.195

COMMAND_PERIOD_S = 0.02



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Проверить, что положительная команда колеса даёт "
            "положительную дельту его энкодера"
        )
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Скорость порта")
    parser.add_argument(
        "--wheel-mps",
        type=float,
        default=0.08,
        help="Скорость одиночного колеса в м/с (по умолчанию 0.08)",
    )
    parser.add_argument(
        "--pulse",
        type=float,
        default=1.0,
        help="Длительность удержания команды в секундах (по умолчанию 1.0)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="Пауза с нулевой командой между тестами (по умолчанию 0.5)",
    )
    return parser.parse_args()




def wheel_command(left_mps: float, right_mps: float) -> bytes:
    """Пересчитать независимые скорости колёс в уставку корпуса.

    Обратная кинематика дифференциального привода в соглашении ROS:
    положительная угловая скорость означает, что правое колесо быстрее.
    """

    linear_mps = 0.5 * (left_mps + right_mps)
    angular_rps = (right_mps - left_mps) / TRACK_WIDTH_M
    return pack_velocity_command(linear_mps, angular_rps)




def drain(port, decoder: FrameDecoder) -> list:
    """Забрать все пакеты телеметрии, уже пришедшие в буфер."""

    packets = []
    waiting = port.in_waiting

    if not waiting:
        return packets

    for message_id, payload in decoder.feed(port.read(waiting)):
        if message_id == MSG_TELEMETRY:
            packets.append(decode_telemetry(payload))

    return packets




def hold(port, decoder: FrameDecoder, left_mps: float, right_mps: float, seconds: float) -> list:
    """Удерживать команду заданное время, собирая телеметрию."""

    packets = []
    command = wheel_command(left_mps, right_mps)

    deadline = time.monotonic() + max(seconds, 0.0)
    next_send = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()

        if now >= next_send:
            port.write(command)
            next_send = now + COMMAND_PERIOD_S

        packets.extend(drain(port, decoder))
        time.sleep(0.002)

    return packets




def sign(value: int) -> int:
    """Вернуть -1, 0 или +1."""

    return (value > 0) - (value < 0)




def run_case(
    port,
    decoder: FrameDecoder,
    wheel: str,
    direction: int,
    wheel_mps: float,
    pulse_s: float,
) -> bool:
    """Проверить один борт в одном направлении."""

    left_command = direction * wheel_mps if wheel == "left" else 0.0
    right_command = direction * wheel_mps if wheel == "right" else 0.0

    drain(port, decoder)
    packets = hold(port, decoder, left_command, right_command, pulse_s)

    if not packets:
        print(f"{wheel:5s} {direction:+d}: телеметрия не получена, ПРОВАЛ")
        return False

    left_sum = sum(packet.left_encoder_delta for packet in packets)
    right_sum = sum(packet.right_encoder_delta for packet in packets)

    tested_sum = left_sum if wheel == "left" else right_sum
    other_sum = right_sum if wheel == "left" else left_sum
    pwm = packets[-1].left_pwm if wheel == "left" else packets[-1].right_pwm

    direction_ok = sign(tested_sum) == direction
    isolation_ok = abs(tested_sum) >= max(2, abs(other_sum))
    passed = direction_ok and isolation_ok

    if passed:
        status = "ОК"
    elif not direction_ok:
        status = f"ПРОВАЛ: инвертировать k{wheel.title()}EncoderReverse"
    else:
        status = "ПРОВАЛ: колесо не изолировано, проверьте TRACK_WIDTH_M"

    print(
        f"{wheel:5s} {direction:+d}: enc={tested_sum:+6d} другое={other_sum:+6d} "
        f"pwm={pwm:+4d} пакетов={len(packets):3d}  {status}"
    )
    return passed




def main() -> int:
    args = parse_args()
    speed = abs(args.wheel_mps)

    port = open_transport(args.port, args.baud)

    decoder = FrameDecoder()

    try:
        print("Поднимите робота над поверхностью перед запуском теста.")
        time.sleep(2.0)
        port.reset_input_buffer()

        # Одометрия и интегралы сбрасываются, чтобы прогон не зависел
        # от того, что робот делал до запуска скрипта.
        port.write(pack_reset_command(RESET_ODOMETRY | RESET_PID))
        hold(port, decoder, 0.0, 0.0, args.settle)

        results = []
        for wheel in ("left", "right"):
            for direction in (+1, -1):
                results.append(
                    run_case(port, decoder, wheel, direction, speed, args.pulse)
                )
                hold(port, decoder, 0.0, 0.0, args.settle)

        print()
        if all(results):
            print("Все проверки пройдены: контракт знаков выполняется.")
            return 0

        print(
            "Есть провалы. Правьте только флаги k*Reverse в "
            "arduino/include/motor_interface.h."
        )
        return 2
    finally:
        hold(port, decoder, 0.0, 0.0, args.settle)
        port.close()



if __name__ == "__main__":
    raise SystemExit(main())
