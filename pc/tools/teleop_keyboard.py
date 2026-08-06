#!/usr/bin/env python3
"""Клавиатурное управление Arduino без ROS 2, протокол v2.

Стрелки задают уставку скорости корпуса, прошивка непрерывно шлёт телеметрию.
Скрипт обязан её вычитывать, иначе входной буфер хоста переполнится прямо
во время управления.

Пример::

  python3 teleop_keyboard.py --port /dev/cu.usbserial-10
"""

from __future__ import annotations

import argparse
import curses
import csv
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "Требуется pyserial. Установка: pip install pyserial"
    ) from exc

sys.path.insert(0, str(Path(__file__).resolve().parent))
# protocol/ - общий кодек, использует и pi/, и pc/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "protocol"))

from rtk_link import (  # noqa: E402
    MSG_TELEMETRY,
    FrameDecoder,
    SequenceTracker,
    Telemetry,
    decode_telemetry,
    describe_telemetry_flags,
    pack_velocity_command,
)

# Обязано совпадать с kTrackWidthM из motor_interface.h. Значение участвует
# только в расчёте ожидаемых скоростей колёс для лога, но расхождение
# сделало бы этот столбец бессмысленным.
TRACK_WIDTH_M = 0.195


@dataclass
class Command:
    """Текущая уставка скорости корпуса."""

    linear_mps: float = 0.0
    angular_rps: float = 0.0


@dataclass
class LogSink:
    """Открытый CSV writer и monotonic начало сеанса."""

    file: object
    writer: csv.writer
    start_monotonic: float


@dataclass
class SerialStats:
    """Состояние UI, последняя измеренная скорость и качество линка."""

    rx_bytes: int = 0
    last_key_code: int = -1
    current_linear_mps: float = 0.0
    current_angular_rps: float = 0.0
    last_flags: int = 0
    decoder: FrameDecoder = field(default_factory=FrameDecoder)
    sequence: SequenceTracker = field(default_factory=SequenceTracker)


@dataclass
class RealtimePlot:
    """Matplotlib-объекты и скользящие временные ряды PID-графика."""

    plt: Any
    axes: tuple[Any, Any]
    lines: tuple[Any, Any, Any, Any]
    start_monotonic: float
    window_s: float
    update_period_s: float
    last_update_s: float = 0.0
    elapsed_s: list[float] = field(default_factory=list)
    target_linear_mps: list[float] = field(default_factory=list)
    current_linear_mps: list[float] = field(default_factory=list)
    target_angular_rps: list[float] = field(default_factory=list)
    current_angular_rps: list[float] = field(default_factory=list)


def open_log_sink(path: Path) -> LogSink:
    """Создать CSV-файл и записать фиксированный заголовок событий."""

    log_file = path.open("w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(
        [
            "timestamp_s",
            "elapsed_s",
            "event",
            "key_code",
            "command_linear_mps",
            "command_angular_rps",
            "target_left_wheel_mps",
            "target_right_wheel_mps",
            "seq",
            "mcu_time_ms",
            "dt_us",
            "current_linear_mps",
            "current_angular_rps",
            "odom_x_m",
            "odom_y_m",
            "odom_heading_rad",
            "left_encoder_delta",
            "right_encoder_delta",
            "left_wheel_rps",
            "right_wheel_rps",
            "left_pwm",
            "right_pwm",
            "sonar_distance_cm",
            "flags",
            "packet_hex",
            "rx_bytes",
        ]
    )
    return LogSink(file=log_file, writer=writer, start_monotonic=time.monotonic())


def parse_args() -> argparse.Namespace:
    """Разобрать скорости, dead-man, Serial, logging и plot options."""

    parser = argparse.ArgumentParser(
        description="Arrow-key teleop for Arduino velocity control"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial device path, for example /dev/cu.usbserial-10",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--linear",
        type=float,
        default=0.30,
        help="Linear speed command for UP/DOWN keys in m/s (default: 0.30)",
    )
    parser.add_argument(
        "--angular",
        type=float,
        default=1.20,
        help="Angular speed command for LEFT/RIGHT keys in rad/s (default: 1.20)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Command publish rate in Hz (default: 20)",
    )
    parser.add_argument(
        "--deadman",
        type=float,
        default=0.45,
        help="Seconds until command auto-resets to zero without key repeat; 0 latches until SPACE (default: 0.45)",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=1.0,
        help="Seconds to wait after opening serial before sending commands (default: 1.0)",
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=Path(f"teleop_pid_{time.strftime('%Y%m%d_%H%M%S')}.csv"),
        help="CSV log path for commands and telemetry (default: teleop_pid_TIMESTAMP.csv)",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Disable CSV logging",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Open a realtime matplotlib window for target/current linear and angular speed",
    )
    parser.add_argument(
        "--plot-window",
        type=float,
        default=20.0,
        help="Realtime plot history window in seconds (default: 20)",
    )
    parser.add_argument(
        "--plot-rate",
        type=float,
        default=10.0,
        help="Realtime plot refresh rate in Hz (default: 10)",
    )
    return parser.parse_args()


def encode_command(cmd: Command) -> bytes:
    """Собрать кадр уставки скорости протокола v2."""

    return pack_velocity_command(cmd.linear_mps, cmd.angular_rps)


def wheel_targets(cmd: Command) -> tuple[float, float]:
    """Вычислить ожидаемые скорости левого и правого колеса для лога."""

    left_mps = cmd.linear_mps - cmd.angular_rps * (TRACK_WIDTH_M * 0.5)
    right_mps = cmd.linear_mps + cmd.angular_rps * (TRACK_WIDTH_M * 0.5)
    return left_mps, right_mps


def open_realtime_plot(window_s: float, update_rate_hz: float) -> RealtimePlot:
    """Создать два интерактивных графика target/current linear/angular."""

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "matplotlib is required for --plot. Install with: pip install -r requirement.txt"
        ) from exc

    plt.ion()
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    if fig.canvas.manager is not None:
        fig.canvas.manager.set_window_title("Teleop PID speeds")

    linear_target_line, = axes[0].plot([], [], label="linear target", color="tab:blue", linestyle="--")
    linear_current_line, = axes[0].plot([], [], label="linear current", color="tab:blue")
    angular_target_line, = axes[1].plot([], [], label="angular target", color="tab:orange", linestyle="--")
    angular_current_line, = axes[1].plot([], [], label="angular current", color="tab:orange")

    axes[0].set_ylabel("linear m/s")
    axes[1].set_ylabel("angular rad/s")
    axes[1].set_xlabel("elapsed seconds")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")

    fig.tight_layout()
    fig.show()

    return RealtimePlot(
        plt=plt,
        axes=(axes[0], axes[1]),
        lines=(linear_target_line, linear_current_line, angular_target_line, angular_current_line),
        start_monotonic=time.monotonic(),
        window_s=max(window_s, 1.0),
        update_period_s=1.0 / max(update_rate_hz, 1.0),
    )


def update_realtime_plot(
    plot: RealtimePlot | None,
    cmd: Command,
    telemetry: Telemetry,
) -> None:
    """Добавить telemetry sample и периодически обновить видимое окно."""

    if plot is None:
        return

    now = time.monotonic()
    elapsed_s = now - plot.start_monotonic
    current_linear_mps = telemetry.current_linear_mps
    current_angular_rps = telemetry.current_angular_rps

    plot.elapsed_s.append(elapsed_s)
    plot.target_linear_mps.append(cmd.linear_mps)
    plot.current_linear_mps.append(current_linear_mps)
    plot.target_angular_rps.append(cmd.angular_rps)
    plot.current_angular_rps.append(current_angular_rps)

    cutoff_s = elapsed_s - plot.window_s
    while plot.elapsed_s and plot.elapsed_s[0] < cutoff_s:
        plot.elapsed_s.pop(0)
        plot.target_linear_mps.pop(0)
        plot.current_linear_mps.pop(0)
        plot.target_angular_rps.pop(0)
        plot.current_angular_rps.pop(0)

    if now - plot.last_update_s < plot.update_period_s:
        return

    (
        linear_target_line,
        linear_current_line,
        angular_target_line,
        angular_current_line,
    ) = plot.lines
    linear_target_line.set_data(plot.elapsed_s, plot.target_linear_mps)
    linear_current_line.set_data(plot.elapsed_s, plot.current_linear_mps)
    angular_target_line.set_data(plot.elapsed_s, plot.target_angular_rps)
    angular_current_line.set_data(plot.elapsed_s, plot.current_angular_rps)

    x_min = max(0.0, elapsed_s - plot.window_s)
    x_max = max(plot.window_s, elapsed_s)
    for ax in plot.axes:
        ax.set_xlim(x_min, x_max)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

    plot.plt.pause(0.001)
    plot.last_update_s = now


def close_realtime_plot(plot: RealtimePlot | None) -> None:
    """Закрыть matplotlib windows, если plot был включён."""

    if plot is None:
        return

    plot.plt.close("all")


def log_row(
    sink: LogSink | None,
    event: str,
    key_code: int,
    cmd: Command,
    packet: bytes = b"",
    stats: SerialStats | None = None,
    telemetry: Telemetry | None = None,
) -> None:
    """Записать унифицированную CSV-строку команды или телеметрии."""

    if sink is None:
        return

    target_left_mps, target_right_mps = wheel_targets(cmd)
    elapsed_s = time.monotonic() - sink.start_monotonic

    # Строки событий команд не несут измерений, поэтому соответствующие
    # столбцы остаются пустыми и не путаются с настоящим нулём.
    if telemetry is None:
        telemetry_columns: list[str] = [""] * 16
    else:
        telemetry_columns = [
            str(telemetry.seq),
            str(telemetry.mcu_time_ms),
            str(telemetry.dt_us),
            f"{telemetry.current_linear_mps:.6f}",
            f"{telemetry.current_angular_rps:.6f}",
            f"{telemetry.odom_x_m:.6f}",
            f"{telemetry.odom_y_m:.6f}",
            f"{telemetry.odom_heading_rad:.6f}",
            str(telemetry.left_encoder_delta),
            str(telemetry.right_encoder_delta),
            f"{telemetry.left_wheel_rps:.6f}",
            f"{telemetry.right_wheel_rps:.6f}",
            str(telemetry.left_pwm),
            str(telemetry.right_pwm),
            str(telemetry.sonar_distance_cm),
            str(telemetry.flags),
        ]

    sink.writer.writerow(
        [
            f"{time.time():.6f}",
            f"{elapsed_s:.6f}",
            event,
            str(key_code),
            f"{cmd.linear_mps:.6f}",
            f"{cmd.angular_rps:.6f}",
            f"{target_left_mps:.6f}",
            f"{target_right_mps:.6f}",
        ]
        + telemetry_columns
        + [
            packet.hex(),
            "" if stats is None else str(stats.rx_bytes),
        ]
    )
    sink.file.flush()


def drain_telemetry(
    ser: serial.Serial,
    sink: LogSink | None,
    cmd: Command,
    stats: SerialStats,
    plot: RealtimePlot | None,
) -> int:
    """Считать доступные байты, разобрать кадры и обновить потребителей."""

    waiting = ser.in_waiting
    if not waiting:
        return 0

    for message_id, payload in stats.decoder.feed(ser.read(waiting)):
        if message_id != MSG_TELEMETRY:
            # Кадры статистики здесь не нужны: за ними есть
            # profile_firmware.py, а UI teleop показывает потери и CRC.
            continue

        telemetry = decode_telemetry(payload)
        stats.sequence.update(telemetry.seq)
        stats.current_linear_mps = telemetry.current_linear_mps
        stats.current_angular_rps = telemetry.current_angular_rps
        stats.last_flags = telemetry.flags

        log_row(
            sink=sink,
            event="rx",
            key_code=-1,
            cmd=cmd,
            stats=stats,
            telemetry=telemetry,
        )
        update_realtime_plot(plot, cmd, telemetry)

    return waiting


def write_command(
    ser: serial.Serial,
    cmd: Command,
    stats: SerialStats,
    log_sink: LogSink | None,
    plot: RealtimePlot | None,
) -> bytes:
    """Передать команду, затем освободить входной Serial-буфер."""

    packet = encode_command(cmd)
    ser.write(packet)
    ser.flush()
    stats.rx_bytes += drain_telemetry(ser, log_sink, cmd, stats, plot)
    return packet


def log_command(
    sink: LogSink | None,
    event: str,
    key_code: int,
    cmd: Command,
    packet: bytes,
    stats: SerialStats,
) -> None:
    """Записать событие передачи уже сформированного packet."""

    log_row(sink, event, key_code, cmd, packet, stats)


def send_stop_command(
    ser: serial.Serial,
    stats: SerialStats,
    log_sink: LogSink | None,
    plot: RealtimePlot | None,
    event: str,
) -> None:
    """Передать нулевую команду при startup/shutdown и залогировать её."""

    stop_cmd = Command()
    packet = write_command(ser, stop_cmd, stats, log_sink, plot)
    log_command(log_sink, event, -1, stop_cmd, packet, stats)


def command_from_key(
    key: int, linear: float, angular: float, current: Command
) -> tuple[Command, bool]:
    """Преобразовать arrows/WASD/Space/Q в новую команду и quit flag."""

    if key == curses.KEY_UP:
        return (
            Command(linear_mps=linear, angular_rps=0.0),
            False,
        )
    if key == curses.KEY_DOWN:
        return (
            Command(linear_mps=-linear, angular_rps=0.0),
            False,
        )
    if key == curses.KEY_LEFT:
        return (
            Command(linear_mps=0.0, angular_rps=angular),
            False,
        )
    if key == curses.KEY_RIGHT:
        return (
            Command(linear_mps=0.0, angular_rps=-angular),
            False,
        )

    # Optional WASD aliases for terminals that don't forward arrow keys reliably.
    if key in (ord("w"), ord("W")):
        return (
            Command(linear_mps=linear, angular_rps=0.0),
            False,
        )
    if key in (ord("s"), ord("S")):
        return (
            Command(linear_mps=-linear, angular_rps=0.0),
            False,
        )
    if key in (ord("a"), ord("A")):
        return (
            Command(linear_mps=0.0, angular_rps=angular),
            False,
        )
    if key in (ord("d"), ord("D")):
        return (
            Command(linear_mps=0.0, angular_rps=-angular),
            False,
        )

    # SPACE force-stops, Q quits.
    if key == ord(" "):
        return Command(), False
    if key in (ord("q"), ord("Q")):
        return current, True

    return current, False


def draw_ui(
    stdscr: curses.window,
    cmd: Command,
    args: argparse.Namespace,
    connected: bool,
    stats: SerialStats,
) -> None:
    """Перерисовать curses UI без блокирующего чтения клавиатуры."""

    stdscr.erase()
    stdscr.addstr(0, 0, "Teleop")
    stdscr.addstr(
        1,
        0,
        f"Port: {args.port} @ {args.baud}  Connected: {'yes' if connected else 'no'}",
    )
    stdscr.addstr(
        2, 0, f"Publish rate: {args.rate:.1f} Hz   Deadman: {args.deadman:.2f} s"
    )
    stdscr.addstr(3, 0, f"UP/DOWN linear: +/-{args.linear:.3f} m/s")
    stdscr.addstr(4, 0, f"LEFT/RIGHT angular: +/-{args.angular:.3f} rad/s")
    stdscr.addstr(
        6,
        0,
        f"Current command -> linear: {cmd.linear_mps:+.3f} m/s   "
        f"angular: {cmd.angular_rps:+.3f} rad/s",
    )
    stdscr.addstr(8, 0, "Controls: arrows or WASD, SPACE stop, Q quit")
    stdscr.addstr(9, 0, f"UART RX bytes drained: {stats.rx_bytes}")
    stdscr.addstr(10, 0, f"Last key code: {stats.last_key_code}")
    stdscr.addstr(
        11,
        0,
        f"Measured speed -> linear: {stats.current_linear_mps:+.3f} m/s   "
        f"angular: {stats.current_angular_rps:+.3f} rad/s",
    )
    # Качество линка выводится прямо в UI: потери и ошибки CRC иначе
    # выглядят просто как более редкая телеметрия.
    stdscr.addstr(
        12,
        0,
        f"Link -> packets: {stats.sequence.received}   "
        f"lost: {stats.sequence.lost} ({100.0 * stats.sequence.loss_ratio:.2f} %)   "
        f"crc: {stats.decoder.bad_crc_count}   junk: {stats.decoder.resync_count}",
    )
    stdscr.addstr(13, 0, f"Firmware flags: {describe_telemetry_flags(stats.last_flags)}")
    stdscr.refresh()


def teleop_loop(
    stdscr: curses.window,
    ser: serial.Serial,
    args: argparse.Namespace,
    log_sink: LogSink | None,
    stats: SerialStats,
    plot: RealtimePlot | None,
) -> int:
    """Выполнять dead-man и периодическую передачу до Q или исключения."""

    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    publish_period = 1.0 / max(args.rate, 1.0)
    deadman = max(args.deadman, 0.0)

    cmd = Command()
    last_key_time = 0.0
    next_publish = time.monotonic()

    while True:
        now = time.monotonic()

        key = stdscr.getch()
        if key != -1:
            cmd, should_quit = command_from_key(key, args.linear, args.angular, cmd)
            last_key_time = now
            stats.last_key_code = key
            packet = encode_command(cmd)
            log_command(log_sink, "key", key, cmd, packet, stats)
            if should_quit:
                cmd = Command()
                packet = write_command(ser, cmd, stats, log_sink, plot)
                log_command(log_sink, "quit", key, cmd, packet, stats)
                return 0

        if deadman > 0.0 and (now - last_key_time) > deadman:
            cmd = Command()

        if now >= next_publish:
            packet = write_command(ser, cmd, stats, log_sink, plot)
            next_publish = now + publish_period
            log_command(log_sink, "tx", -1, cmd, packet, stats)

        draw_ui(stdscr, cmd, args, connected=True, stats=stats)
        time.sleep(0.005)


def main() -> int:
    """Открыть ресурсы teleop и гарантированно отправить stop при выходе."""

    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.2)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    log_sink = None if args.no_log else open_log_sink(args.log)
    plot = open_realtime_plot(args.plot_window, args.plot_rate) if args.plot else None
    stats = SerialStats()

    time.sleep(max(args.startup_delay, 0.0))
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Поток команд начинается с остановки.
    startup_cmd = Command()
    startup_packet = write_command(ser, startup_cmd, stats, log_sink, plot)
    log_command(log_sink, "startup", -1, startup_cmd, startup_packet, stats)

    exit_code = 0
    shutdown_event = "shutdown"
    try:
        exit_code = curses.wrapper(lambda stdscr: teleop_loop(stdscr, ser, args, log_sink, stats, plot))
    except KeyboardInterrupt:
        shutdown_event = "keyboard_interrupt"
        exit_code = 130
    finally:
        try:
            send_stop_command(
                ser,
                stats,
                log_sink,
                plot,
                shutdown_event,
            )
        except Exception as exc:
            print(f"Не удалось отправить стоп при завершении: {exc}", file=sys.stderr)
        finally:
            try:
                ser.close()
            finally:
                close_realtime_plot(plot)
                if log_sink is not None:
                    log_sink.file.close()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
