#!/usr/bin/env python3
"""Чтение и запись телеметрии Arduino по протоколу v2.

Инструмент показывает поток как есть: одну строку на пакет, с номером
пакета, фактическим периодом цикла и взведёнными флагами. Для сводных
метрик джиттера и потерь есть отдельный ``profile_firmware.py``.

Примеры::

    python3 log_telemetry.py --port /dev/cu.usbserial-10
    python3 log_telemetry.py --port /dev/ttyUSB0 --out run.csv
    python3 log_telemetry.py --port /dev/ttyUSB0 --stats
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
# protocol/ - общий кодек, использует и pi/, и pc/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "protocol"))

from transport import open_transport  # noqa: E402

from rtk_link import (  # noqa: E402
    MSG_STATS,
    MSG_TELEMETRY,
    FrameDecoder,
    SequenceTracker,
    decode_stats,
    decode_telemetry,
    describe_telemetry_flags,
)

CSV_COLUMNS = [
    "host_time_s",
    "seq",
    "mcu_time_ms",
    "dt_us",
    "left_encoder_delta",
    "right_encoder_delta",
    "left_wheel_rps",
    "right_wheel_rps",
    "left_pwm",
    "right_pwm",
    "odom_x_m",
    "odom_y_m",
    "odom_heading_rad",
    "odom_heading_deg",
    "current_linear_mps",
    "current_angular_rps",
    "sonar_distance_cm",
    "flags",
]



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Печатать и записывать телеметрию Arduino протокола v2"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial-устройство или host:port сервера link_server.py",
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Скорость порта (по умолчанию 115200)"
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=10.0,
        help="Ограничение частоты печати в Гц (по умолчанию 10)",
    )
    parser.add_argument(
        "--stats",
        action="store_true",
        help="Печатать также пакеты статистики прошивки",
    )
    parser.add_argument("--out", type=Path, help="Путь к CSV-файлу")
    return parser.parse_args()




def format_telemetry(telemetry) -> str:
    """Собрать одну компактную человекочитаемую строку."""

    heading_deg = telemetry.odom_heading_rad * 180.0 / math.pi

    return (
        f"#{telemetry.seq:5d} "
        f"t={telemetry.mcu_time_ms / 1000.0:8.3f}с "
        f"dt={telemetry.dt_us / 1000.0:6.2f}мс "
        f"odom=(x={telemetry.odom_x_m: .3f} y={telemetry.odom_y_m: .3f} "
        f"yaw={telemetry.odom_heading_rad: .3f}рад/{heading_deg: .1f}°) "
        f"v=({telemetry.current_linear_mps:+.3f}м/с {telemetry.current_angular_rps:+.3f}рад/с) "
        f"enc=(L{telemetry.left_encoder_delta:+5d} R{telemetry.right_encoder_delta:+5d}) "
        f"rps=(L{telemetry.left_wheel_rps:+6.2f} R{telemetry.right_wheel_rps:+6.2f}) "
        f"pwm=(L{telemetry.left_pwm:+4d} R{telemetry.right_pwm:+4d}) "
        f"sonar={telemetry.sonar_distance_cm:+4d}см "
        f"[{describe_telemetry_flags(telemetry.flags)}]"
    )




def format_stats(stats) -> str:
    return (
        f"  СТАТИСТИКА  dt=[{stats.dt_min_us / 1000.0:.2f} "
        f"{stats.dt_mean_us / 1000.0:.2f} {stats.dt_max_us / 1000.0:.2f}]мс "
        f"цикл_макс={stats.cycle_duration_max_us / 1000.0:.2f}мс "
        f"сонар_макс={stats.sonar_block_max_us / 1000.0:.2f}мс "
        f"срывов={stats.overruns} tx_drop={stats.tx_dropped} "
        f"crc={stats.rx_bad_crc} ram={stats.free_ram_bytes}"
    )




def main() -> int:
    args = parse_args()
    min_period = 1.0 / max(args.hz, 0.1)

    port = open_transport(args.port, args.baud)

    log_file = None
    csv_writer = None
    if args.out:
        log_file = args.out.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(log_file)
        csv_writer.writerow(CSV_COLUMNS)

    decoder = FrameDecoder()
    sequence = SequenceTracker()

    print(f"Порт {args.port} @ {args.baud}, протокол v2")
    if args.out:
        print(f"Запись в {args.out}")

    time.sleep(2.0)
    port.reset_input_buffer()

    start = time.monotonic()
    last_print = 0.0

    try:
        while True:
            waiting = port.in_waiting

            if not waiting:
                # Пауза короче периода телеметрии, чтобы не терять пакеты
                # и при этом не занимать ядро вхолостую.
                time.sleep(0.002)
                continue

            for message_id, payload in decoder.feed(port.read(waiting)):
                now = time.monotonic()

                if message_id == MSG_TELEMETRY:
                    telemetry = decode_telemetry(payload)
                    missing = sequence.update(telemetry.seq)

                    if missing:
                        print(f"  ! потеряно пакетов: {missing}")

                    if csv_writer is not None:
                        heading_deg = telemetry.odom_heading_rad * 180.0 / math.pi
                        csv_writer.writerow(
                            [
                                f"{now - start:.6f}",
                                telemetry.seq,
                                telemetry.mcu_time_ms,
                                telemetry.dt_us,
                                telemetry.left_encoder_delta,
                                telemetry.right_encoder_delta,
                                f"{telemetry.left_wheel_rps:.6f}",
                                f"{telemetry.right_wheel_rps:.6f}",
                                telemetry.left_pwm,
                                telemetry.right_pwm,
                                f"{telemetry.odom_x_m:.6f}",
                                f"{telemetry.odom_y_m:.6f}",
                                f"{telemetry.odom_heading_rad:.6f}",
                                f"{heading_deg:.6f}",
                                f"{telemetry.current_linear_mps:.6f}",
                                f"{telemetry.current_angular_rps:.6f}",
                                telemetry.sonar_distance_cm,
                                telemetry.flags,
                            ]
                        )
                        if log_file is not None:
                            log_file.flush()

                    if now - last_print >= min_period:
                        print(format_telemetry(telemetry))
                        last_print = now

                elif message_id == MSG_STATS and args.stats:
                    print(format_stats(decode_stats(payload)))

    except KeyboardInterrupt:
        print("\nОстановлено.")
        print(
            f"Принято {sequence.received}, потеряно {sequence.lost} "
            f"({100.0 * sequence.loss_ratio:.2f} %), "
            f"ошибок CRC {decoder.bad_crc_count}, "
            f"мусорных байт {decoder.resync_count}"
        )
        return 0
    finally:
        port.close()
        if log_file is not None:
            log_file.close()



if __name__ == "__main__":
    raise SystemExit(main())
