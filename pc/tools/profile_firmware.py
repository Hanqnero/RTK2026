#!/usr/bin/env python3
"""Профилировщик прошивки: джиттер управляющего цикла, потери и стоимость сонара.

Инструмент отвечает на вопросы, на которые протокол v1 ответить не мог:

* каков фактический период управляющего цикла и его разброс;
* сколько пакетов теряется и где именно - в прошивке или в USB;
* сколько микросекунд занимает измерение сонара;
* доходят ли команды до MCU без повреждений.

Инструмент печатает измеренные величины и не делает выводов о том,
какие из них приемлемы: это зависит от задачи и от режима работы.

Джиттер прошивки и джиттер транспорта разделяются: первый берётся из
``dt_us``, измеренного самим MCU, второй - из разности между временем
прибытия пакета на хост и временем MCU в этом же пакете.

Примеры::

    python3 profile_firmware.py --port /dev/cu.usbserial-10
    python3 profile_firmware.py --port /dev/ttyUSB0 --duration 30 --csv run.csv
    python3 profile_firmware.py --port /dev/ttyUSB0 --linear 0.15 --duration 20
"""

from __future__ import annotations

import argparse
import csv
import statistics
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
    RESET_STATS,
    FrameDecoder,
    SequenceTracker,
    decode_stats,
    decode_telemetry,
    describe_telemetry_flags,
    pack_reset_command,
    pack_velocity_command,
)

COMMAND_PERIOD_S = 0.02



def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Измерить джиттер управляющего цикла, потери пакетов и "
            "стоимость сонара по телеметрии протокола v2."
        )
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
        "--duration",
        type=float,
        default=20.0,
        help="Длительность профилирования в секундах (по умолчанию 20)",
    )
    parser.add_argument(
        "--linear",
        type=float,
        default=0.0,
        help=(
            "Линейная уставка м/с на время замера. Ненулевое значение "
            "приводит колёса в движение"
        ),
    )
    parser.add_argument(
        "--angular",
        type=float,
        default=0.0,
        help="Угловая уставка рад/с на время замера",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="Записать телеметрию пакет за пакетом в CSV",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Не печатать текущее состояние, только итоговый отчёт",
    )
    return parser.parse_args()




def percentile(values: list[float], fraction: float) -> float:
    """Перцентиль по ближайшему рангу, без интерполяции."""

    if not values:
        return 0.0

    ordered = sorted(values)
    index = min(
        len(ordered) - 1,
        max(0, int(round(fraction * (len(ordered) - 1)))),
    )
    return ordered[index]




def format_microseconds(value: float) -> str:
    return f"{value / 1000.0:8.2f} мс"




class Profiler:
    """Накопитель измерений за один прогон."""

    def __init__(self) -> None:
        self.dt_samples_us: list[int] = []
        # Разность между временем прибытия на хост и временем MCU. Абсолютное
        # значение смысла не имеет, интерес представляет только разброс.
        self.transport_offsets_ms: list[float] = []

        self.sequence = SequenceTracker()
        self.telemetry_count = 0
        self.first_telemetry_monotonic: float | None = None
        self.last_telemetry_monotonic: float | None = None

        self.flag_counts: dict[str, int] = {
            "sonar_stop": 0,
            "cmd_timeout": 0,
            "sat_left": 0,
            "sat_right": 0,
            "overrun": 0,
        }

        self.latest_stats = None
        self.stats_count = 0
        self.sonar_block_max_us = 0
        self.window_dt_max_us = 0


    def add_telemetry(self, telemetry, arrival_monotonic: float) -> None:
        self.telemetry_count += 1
        self.sequence.update(telemetry.seq)

        if self.first_telemetry_monotonic is None:
            self.first_telemetry_monotonic = arrival_monotonic
        self.last_telemetry_monotonic = arrival_monotonic

        # Первый пакет содержит dt от setup() до первого цикла и не отражает
        # установившийся режим, поэтому в статистику периода не идёт.
        if self.telemetry_count > 1:
            self.dt_samples_us.append(telemetry.dt_us)

        self.transport_offsets_ms.append(
            arrival_monotonic * 1000.0 - telemetry.mcu_time_ms
        )

        if telemetry.flags & 0x01:
            self.flag_counts["sonar_stop"] += 1
        if telemetry.flags & 0x02:
            self.flag_counts["cmd_timeout"] += 1
        if telemetry.flags & 0x04:
            self.flag_counts["sat_left"] += 1
        if telemetry.flags & 0x08:
            self.flag_counts["sat_right"] += 1
        if telemetry.flags & 0x10:
            self.flag_counts["overrun"] += 1



    def add_stats(self, stats) -> None:
        self.latest_stats = stats
        self.stats_count += 1
        self.sonar_block_max_us = max(self.sonar_block_max_us, stats.sonar_block_max_us)
        self.window_dt_max_us = max(self.window_dt_max_us, stats.dt_max_us)





def print_report(profiler: Profiler, decoder: FrameDecoder, duration_s: float) -> None:
    print()

    if profiler.telemetry_count == 0:
        print(f"телеметрия не получена, мусорных байт {decoder.resync_count}")
        return

    samples = profiler.dt_samples_us

    print()
    print(f"длительность {duration_s:.2f} с, телеметрии {profiler.telemetry_count}, "
          f"статистики {profiler.stats_count}")

    if profiler.first_telemetry_monotonic and profiler.last_telemetry_monotonic:
        span = profiler.last_telemetry_monotonic - profiler.first_telemetry_monotonic
        if span > 0:
            rate = (profiler.telemetry_count - 1) / span
            print(f"частота {rate:.2f} Гц")

    print()
    print("период цикла (по данным MCU)")

    if samples:
        mean_us = statistics.fmean(samples)
        print(f"  минимум                   {format_microseconds(min(samples))}")
        print(f"  среднее                   {format_microseconds(mean_us)}")
        print(f"  медиана                   {format_microseconds(percentile(samples, 0.50))}")
        print(f"  95-й перцентиль           {format_microseconds(percentile(samples, 0.95))}")
        print(f"  99-й перцентиль           {format_microseconds(percentile(samples, 0.99))}")
        print(f"  максимум                  {format_microseconds(max(samples))}")

        spread_us = max(samples) - min(samples)
        print(f"  размах                    {format_microseconds(spread_us)}")

        if len(samples) > 1:
            deviation_us = statistics.pstdev(samples)
            print(f"  СКО                       {format_microseconds(deviation_us)}")
            print(
                f"  СКО от среднего           {100.0 * deviation_us / mean_us:8.2f} %"
            )

    print()
    print("джиттер транспорта MCU -> хост")

    offsets = profiler.transport_offsets_ms
    if len(offsets) > 1:
        # Абсолютное смещение включает произвольную разницу нулей часов,
        # поэтому осмысленна только вариация: она и есть задержка USB и
        # планировщика поверх времени прошивки.
        print(f"  размах задержки           {max(offsets) - min(offsets):8.2f} мс")
        print(f"  СКО задержки              {statistics.pstdev(offsets):8.2f} мс")

    print()
    print("поток")
    print(f"  потеряно пакетов          {profiler.sequence.lost:8d}")
    print(f"  доля потерь               {100.0 * profiler.sequence.loss_ratio:8.3f} %")
    print(f"  не по порядку             {profiler.sequence.reordered:8d}")
    print(f"  кадров принято            {decoder.frame_count:8d}")
    print(f"  ошибок CRC                {decoder.bad_crc_count:8d}")
    print(f"  мусорных байт             {decoder.resync_count:8d}")
    print(f"  битых длин                {decoder.bad_length_count:8d}")

    print()
    print("флаги телеметрии")
    for name, count in profiler.flag_counts.items():
        share = 100.0 * count / profiler.telemetry_count
        print(f"  {name:<24}  {count:8d}   {share:6.2f} %")

    stats = profiler.latest_stats
    if stats is not None:
        print()
        print("счётчики прошивки")
        print(f"  версия протокола          {stats.protocol_version:8d}")
        print(f"  аптайм                    {stats.uptime_ms / 1000.0:8.1f} с")
        print(f"  управляющих циклов        {stats.control_cycles:8d}")
        print(f"  макс. расчёт цикла        {format_microseconds(stats.cycle_duration_max_us)}")
        print(f"  макс. блокировка сонара   {format_microseconds(profiler.sonar_block_max_us)}")
        print(f"  срывов периода            {stats.overruns:8d}")
        print(f"  кадров отправлено         {stats.tx_frames:8d}")
        print(f"  кадров потеряно на TX     {stats.tx_dropped:8d}")
        print(f"  кадров принято MCU        {stats.rx_frames:8d}")
        print(f"  ошибок CRC на MCU         {stats.rx_bad_crc:8d}")
        print(f"  мусорных байт на MCU      {stats.rx_resync:8d}")
        print(f"  свободной RAM             {stats.free_ram_bytes:8d} байт")

    print()





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
    "current_linear_mps",
    "current_angular_rps",
    "sonar_distance_cm",
    "flags",
]



def main() -> int:
    args = parse_args()

    if args.linear != 0.0 or args.angular != 0.0:
        print(
            f"ВНИМАНИЕ: замер пойдёт под нагрузкой "
            f"linear={args.linear} м/с, angular={args.angular} рад/с. "
            "Колёса будут вращаться."
        )

    port = open_transport(args.port, args.baud)

    decoder = FrameDecoder()
    profiler = Profiler()

    csv_file = None
    csv_writer = None
    if args.csv:
        csv_file = args.csv.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(CSV_COLUMNS)

    print(f"Порт {args.port} @ {args.baud}")
    print("Ожидание перезагрузки Arduino после открытия порта...")
    time.sleep(2.0)
    port.reset_input_buffer()

    # Счётчики прошивки обнуляются, чтобы отчёт относился к этому прогону,
    # а не ко всему времени с момента включения питания.
    port.write(pack_reset_command(RESET_STATS))

    print(f"Замер {args.duration:.1f} с. Прерывание: Ctrl+C")
    print()

    start_monotonic = time.monotonic()
    deadline = start_monotonic + args.duration
    next_command_at = start_monotonic
    last_print_at = 0.0

    try:
        while True:
            now = time.monotonic()

            if now >= deadline:
                break

            if now >= next_command_at:
                port.write(pack_velocity_command(args.linear, args.angular))
                next_command_at += COMMAND_PERIOD_S

            waiting = port.in_waiting
            if waiting:
                for message_id, payload in decoder.feed(port.read(waiting)):
                    arrival = time.monotonic()

                    if message_id == MSG_TELEMETRY:
                        telemetry = decode_telemetry(payload)
                        profiler.add_telemetry(telemetry, arrival)

                        if csv_writer is not None:
                            csv_writer.writerow(
                                [
                                    f"{arrival - start_monotonic:.6f}",
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
                                    f"{telemetry.current_linear_mps:.6f}",
                                    f"{telemetry.current_angular_rps:.6f}",
                                    telemetry.sonar_distance_cm,
                                    telemetry.flags,
                                ]
                            )

                    elif message_id == MSG_STATS:
                        profiler.add_stats(decode_stats(payload))

            if not args.quiet and now - last_print_at >= 1.0:
                last_print_at = now
                remaining = deadline - now
                latest = profiler.latest_stats
                dt_text = (
                    f"dt={latest.dt_mean_us / 1000.0:.1f}мс "
                    f"макс={latest.dt_max_us / 1000.0:.1f}мс"
                    if latest is not None
                    else "dt=?"
                )
                print(
                    f"  осталось {remaining:5.1f} с  "
                    f"пакетов={profiler.telemetry_count:6d}  "
                    f"потеряно={profiler.sequence.lost:4d}  "
                    f"crc={decoder.bad_crc_count:3d}  {dt_text}"
                )

            # Пауза короче периода телеметрии: цикл не должен становиться
            # узким местом измерения, которое сам же и проводит.
            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\nПрервано пользователем.")
    finally:
        # Останавливаем приводы независимо от причины выхода.
        try:
            port.write(pack_velocity_command(0.0, 0.0))
            time.sleep(0.1)
        except OSError:
            pass

        port.close()

        if csv_file is not None:
            csv_file.close()

    measured_duration = time.monotonic() - start_monotonic
    print_report(profiler, decoder, measured_duration)

    if args.csv:
        print(f"Телеметрия сохранена: {args.csv}")

    return 0



if __name__ == "__main__":
    raise SystemExit(main())
