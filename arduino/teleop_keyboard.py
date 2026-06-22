#!/usr/bin/env python3
"""Keyboard teleoperation for the Arduino control interface in src/main.cpp.

Arrow keys publish velocity commands as packed little-endian values:
    ControlPacket: <ffB = (target_linear_mps, target_angular_rps, debug_raw_encoder)

The firmware also emits TelemetryPacket records continuously; this script drains
them so the host serial input buffer does not fill while teleoperating.

Usage example:
  python3 teleop_keyboard.py --port /dev/cu.usbserial-10
"""

from __future__ import annotations

import argparse
import curses
import csv
import math
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install with: pip install pyserial"
    ) from exc


CONTROL_PACKET = struct.Struct("<ffB")
TELEMETRY_PACKET = struct.Struct("<fffiihh")

CONTROL_PERIOD_MS = 100
WHEEL_RADIUS_M = 0.024
TRACK_WIDTH_M = 0.040
ENCODER_COUNTS_PER_MOTOR_REV = 1400.0
GEARBOX_RATIO = 18.1
WHEEL_CIRCUMFERENCE_M = 2.0 * math.pi * WHEEL_RADIUS_M


@dataclass
class Command:
    linear_mps: float = 0.0
    angular_rps: float = 0.0
    debug_raw_encoder: int = 0


@dataclass
class LogSink:
    file: object
    writer: csv.writer
    start_monotonic: float


@dataclass
class SerialStats:
    rx_bytes: int = 0
    last_key_code: int = -1
    telemetry_buffer: bytearray = field(default_factory=bytearray)


@dataclass
class RealtimePlot:
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
            "debug_raw_encoder",
            "target_left_wheel_mps",
            "target_right_wheel_mps",
            "current_left_wheel_mps",
            "current_right_wheel_mps",
            "odom_x_m",
            "odom_y_m",
            "odom_heading_rad",
            "raw_left_encoder_delta",
            "raw_right_encoder_delta",
            "left_pwm",
            "right_pwm",
            "packet_hex",
            "rx_bytes",
        ]
    )
    return LogSink(file=log_file, writer=writer, start_monotonic=time.monotonic())


def parse_args() -> argparse.Namespace:
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
        "--debug-raw-encoder",
        action="store_true",
        help="Ask firmware to include raw encoder deltas in telemetry",
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
    return CONTROL_PACKET.pack(cmd.linear_mps, cmd.angular_rps, cmd.debug_raw_encoder)


def wheel_targets(cmd: Command) -> tuple[float, float]:
    left_mps = cmd.linear_mps - cmd.angular_rps * (TRACK_WIDTH_M * 0.5)
    right_mps = cmd.linear_mps + cmd.angular_rps * (TRACK_WIDTH_M * 0.5)
    return left_mps, right_mps


def encoder_delta_to_wheel_mps(delta: int) -> float:
    counts_per_second = delta * (1000.0 / CONTROL_PERIOD_MS)
    motor_rps = counts_per_second / ENCODER_COUNTS_PER_MOTOR_REV
    wheel_rps = motor_rps / GEARBOX_RATIO
    return wheel_rps * WHEEL_CIRCUMFERENCE_M


def open_realtime_plot(window_s: float, update_rate_hz: float) -> RealtimePlot:
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
    telemetry: tuple[float, float, float, int, int, int, int],
) -> None:
    if plot is None:
        return

    now = time.monotonic()
    elapsed_s = now - plot.start_monotonic
    current_left_mps = encoder_delta_to_wheel_mps(telemetry[3])
    current_right_mps = encoder_delta_to_wheel_mps(telemetry[4])
    current_linear_mps = 0.5 * (current_left_mps + current_right_mps)
    current_angular_rps = (current_right_mps - current_left_mps) / TRACK_WIDTH_M

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
    telemetry: tuple[float, float, float, int, int, int, int] | None = None,
) -> None:
    if sink is None:
        return

    target_left_mps, target_right_mps = wheel_targets(cmd)
    elapsed_s = time.monotonic() - sink.start_monotonic

    current_left_mps = ""
    current_right_mps = ""
    odom_x_m = ""
    odom_y_m = ""
    odom_heading_rad = ""
    raw_left_encoder_delta = ""
    raw_right_encoder_delta = ""
    left_pwm = ""
    right_pwm = ""

    if telemetry is not None:
        (
            odom_x_m,
            odom_y_m,
            odom_heading_rad,
            raw_left_encoder_delta,
            raw_right_encoder_delta,
            left_pwm,
            right_pwm,
        ) = telemetry
        current_left_mps = encoder_delta_to_wheel_mps(raw_left_encoder_delta)
        current_right_mps = encoder_delta_to_wheel_mps(raw_right_encoder_delta)

    sink.writer.writerow(
        [
            f"{time.time():.6f}",
            f"{elapsed_s:.6f}",
            event,
            str(key_code),
            f"{cmd.linear_mps:.6f}",
            f"{cmd.angular_rps:.6f}",
            str(cmd.debug_raw_encoder),
            f"{target_left_mps:.6f}",
            f"{target_right_mps:.6f}",
            "" if current_left_mps == "" else f"{current_left_mps:.6f}",
            "" if current_right_mps == "" else f"{current_right_mps:.6f}",
            "" if odom_x_m == "" else f"{odom_x_m:.6f}",
            "" if odom_y_m == "" else f"{odom_y_m:.6f}",
            "" if odom_heading_rad == "" else f"{odom_heading_rad:.6f}",
            str(raw_left_encoder_delta),
            str(raw_right_encoder_delta),
            str(left_pwm),
            str(right_pwm),
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
    waiting = ser.in_waiting
    if waiting:
        chunk = ser.read(waiting)
        stats.telemetry_buffer.extend(chunk)
        while len(stats.telemetry_buffer) >= TELEMETRY_PACKET.size:
            raw = bytes(stats.telemetry_buffer[: TELEMETRY_PACKET.size])
            del stats.telemetry_buffer[: TELEMETRY_PACKET.size]
            telemetry = TELEMETRY_PACKET.unpack(raw)
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
    log_row(sink, event, key_code, cmd, packet, stats)


def send_stop_command(
    ser: serial.Serial,
    debug_raw_encoder: int,
    stats: SerialStats,
    log_sink: LogSink | None,
    plot: RealtimePlot | None,
    event: str,
) -> None:
    stop_cmd = Command(debug_raw_encoder=debug_raw_encoder)
    packet = write_command(ser, stop_cmd, stats, log_sink, plot)
    log_command(log_sink, event, -1, stop_cmd, packet, stats)


def command_from_key(
    key: int, linear: float, angular: float, current: Command
) -> tuple[Command, bool]:
    if key == curses.KEY_UP:
        return (
            Command(
                linear_mps=linear,
                angular_rps=0.0,
                debug_raw_encoder=current.debug_raw_encoder,
            ),
            False,
        )
    if key == curses.KEY_DOWN:
        return (
            Command(
                linear_mps=-linear,
                angular_rps=0.0,
                debug_raw_encoder=current.debug_raw_encoder,
            ),
            False,
        )
    if key == curses.KEY_LEFT:
        return (
            Command(
                linear_mps=0.0,
                angular_rps=angular,
                debug_raw_encoder=current.debug_raw_encoder,
            ),
            False,
        )
    if key == curses.KEY_RIGHT:
        return (
            Command(
                linear_mps=0.0,
                angular_rps=-angular,
                debug_raw_encoder=current.debug_raw_encoder,
            ),
            False,
        )

    # Optional WASD aliases for terminals that don't forward arrow keys reliably.
    if key in (ord("w"), ord("W")):
        return (
            Command(
                linear_mps=linear,
                angular_rps=0.0,
                debug_raw_encoder=current.debug_raw_encoder,
            ),
            False,
        )
    if key in (ord("s"), ord("S")):
        return (
            Command(
                linear_mps=-linear,
                angular_rps=0.0,
                debug_raw_encoder=current.debug_raw_encoder,
            ),
            False,
        )
    if key in (ord("a"), ord("A")):
        return (
            Command(
                linear_mps=0.0,
                angular_rps=angular,
                debug_raw_encoder=current.debug_raw_encoder,
            ),
            False,
        )
    if key in (ord("d"), ord("D")):
        return (
            Command(
                linear_mps=0.0,
                angular_rps=-angular,
                debug_raw_encoder=current.debug_raw_encoder,
            ),
            False,
        )

    # SPACE force-stops, Q quits.
    if key == ord(" "):
        return Command(debug_raw_encoder=current.debug_raw_encoder), False
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
        f"Current command -> linear: {cmd.linear_mps:+.3f} m/s   angular: {cmd.angular_rps:+.3f} rad/s   debug_raw_encoder: {cmd.debug_raw_encoder}",
    )
    stdscr.addstr(8, 0, "Controls: arrows or WASD, SPACE stop, Q quit")
    stdscr.addstr(
        9, 0, f"Debug raw encoder: {'on' if cmd.debug_raw_encoder else 'off'}"
    )
    stdscr.addstr(10, 0, f"UART RX bytes drained: {stats.rx_bytes}")
    stdscr.addstr(11, 0, f"Last key code: {stats.last_key_code}")
    stdscr.refresh()


def teleop_loop(
    stdscr: curses.window,
    ser: serial.Serial,
    args: argparse.Namespace,
    log_sink: LogSink | None,
    stats: SerialStats,
    plot: RealtimePlot | None,
) -> int:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    publish_period = 1.0 / max(args.rate, 1.0)
    deadman = max(args.deadman, 0.0)

    cmd = Command()
    cmd.debug_raw_encoder = 1 if args.debug_raw_encoder else 0
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
                cmd = Command(debug_raw_encoder=cmd.debug_raw_encoder)
                packet = write_command(ser, cmd, stats, log_sink, plot)
                log_command(log_sink, "quit", key, cmd, packet, stats)
                return 0

        if deadman > 0.0 and (now - last_key_time) > deadman:
            cmd = Command(debug_raw_encoder=cmd.debug_raw_encoder)

        if now >= next_publish:
            packet = write_command(ser, cmd, stats, log_sink, plot)
            next_publish = now + publish_period
            log_command(log_sink, "tx", -1, cmd, packet, stats)

        draw_ui(stdscr, cmd, args, connected=True, stats=stats)
        time.sleep(0.005)


def main() -> int:
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.2)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    log_sink = None if args.no_log else open_log_sink(args.log)
    plot = open_realtime_plot(args.plot_window, args.plot_rate) if args.plot else None
    if log_sink is not None or plot is not None:
        args.debug_raw_encoder = True
    stats = SerialStats()

    time.sleep(max(args.startup_delay, 0.0))
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Reset command stream to stop on startup.
    debug_raw_encoder = 1 if args.debug_raw_encoder else 0
    startup_cmd = Command(debug_raw_encoder=debug_raw_encoder)
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
                startup_cmd.debug_raw_encoder,
                stats,
                log_sink,
                plot,
                shutdown_event,
            )
        except Exception as exc:
            print(f"Failed to send stop command during shutdown: {exc}", file=sys.stderr)
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
