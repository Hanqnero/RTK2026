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
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install with: pip install pyserial"
    ) from exc


CONTROL_PACKET = struct.Struct("<ffB")
TELEMETRY_PACKET = struct.Struct("<fffiihh")


@dataclass
class Command:
    linear_mps: float = 0.0
    angular_rps: float = 0.0
    debug_raw_encoder: int = 0


@dataclass
class LogSink:
    file: object
    writer: csv.writer


@dataclass
class SerialStats:
    rx_bytes: int = 0
    last_key_code: int = -1


def open_log_sink(path: Path) -> LogSink:
    log_file = path.open("w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(
        [
            "timestamp_s",
            "event",
            "key_code",
            "linear_mps",
            "angular_rps",
            "debug_raw_encoder",
            "packet_hex",
            "rx_bytes",
        ]
    )
    return LogSink(file=log_file, writer=writer)


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
        help="Optional CSV log file for transmitted control packets",
    )
    return parser.parse_args()


def encode_command(cmd: Command) -> bytes:
    return CONTROL_PACKET.pack(cmd.linear_mps, cmd.angular_rps, cmd.debug_raw_encoder)


def drain_telemetry(ser: serial.Serial) -> int:
    waiting = ser.in_waiting
    if waiting:
        ser.read(waiting)
    return waiting


def write_command(ser: serial.Serial, cmd: Command, stats: SerialStats) -> bytes:
    packet = encode_command(cmd)
    ser.write(packet)
    ser.flush()
    stats.rx_bytes += drain_telemetry(ser)
    return packet


def log_command(
    sink: LogSink | None,
    event: str,
    key_code: int,
    cmd: Command,
    packet: bytes,
    stats: SerialStats,
) -> None:
    if sink is None:
        return

    sink.writer.writerow(
        [
            f"{time.time():.6f}",
            event,
            str(key_code),
            f"{cmd.linear_mps:.6f}",
            f"{cmd.angular_rps:.6f}",
            str(cmd.debug_raw_encoder),
            packet.hex(),
            str(stats.rx_bytes),
        ]
    )
    sink.file.flush()


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
                packet = write_command(ser, cmd, stats)
                log_command(log_sink, "quit", key, cmd, packet, stats)
                return 0

        if deadman > 0.0 and (now - last_key_time) > deadman:
            cmd = Command(debug_raw_encoder=cmd.debug_raw_encoder)

        if now >= next_publish:
            packet = write_command(ser, cmd, stats)
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

    log_sink = open_log_sink(args.log) if args.log else None
    stats = SerialStats()

    time.sleep(max(args.startup_delay, 0.0))
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Reset command stream to stop on startup.
    startup_cmd = Command(debug_raw_encoder=1 if args.debug_raw_encoder else 0)
    startup_packet = write_command(ser, startup_cmd, stats)
    log_command(log_sink, "startup", -1, startup_cmd, startup_packet, stats)

    try:
        return curses.wrapper(lambda stdscr: teleop_loop(stdscr, ser, args, log_sink, stats))
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            stop_cmd = Command(debug_raw_encoder=startup_cmd.debug_raw_encoder)
            stop_packet = write_command(ser, stop_cmd, stats)
            log_command(log_sink, "shutdown", -1, stop_cmd, stop_packet, stats)
        except Exception:
            pass
        ser.close()
        if log_sink is not None:
            log_sink.file.close()


if __name__ == "__main__":
    raise SystemExit(main())
