#!/usr/bin/env python3
"""Simple control-firmware exercise script.

Sequence:
  - left motor forward, then reverse
  - right motor forward, then reverse
  - both motors forward for 2 seconds
  - both motors reverse for 2 seconds

The script sends ControlPacket values matching include/control_protocol.h and
logs both transmitted control events and received telemetry packets to a CSV
file.
"""

from __future__ import annotations

import argparse
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
TELEMETRY_PACKET = struct.Struct("<fffiihhff")


@dataclass(frozen=True)
class Command:
    label: str
    linear_mps: float
    angular_rps: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a simple motor direction test and log telemetry"
    )
    parser.add_argument("--port", required=True, help="Serial device path")
    parser.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(f"motor_test_{time.strftime('%Y%m%d_%H%M%S')}.csv"),
        help="CSV log file path",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Seconds to hold each motion command (default: 2.0)",
    )
    parser.add_argument(
        "--debug-raw-encoder",
        action="store_true",
        help="Ask firmware to include raw encoder deltas in telemetry",
    )
    return parser.parse_args()


def encode_command(linear_mps: float, angular_rps: float, debug_raw_encoder: int) -> bytes:
    return CONTROL_PACKET.pack(linear_mps, angular_rps, debug_raw_encoder)


def write_log_row(writer: csv.writer, row_type: str, **fields: object) -> None:
    writer.writerow([row_type] + [fields[k] for k in fields])


def main() -> int:
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    commands = [
        Command("left_forward", +0.20, 0.0),
        Command("left_reverse", -0.20, 0.0),
        Command("right_forward", 0.0, +1.50),
        Command("right_reverse", 0.0, -1.50),
        Command("both_forward", +0.20, 0.0),
        Command("both_reverse", -0.20, 0.0),
    ]

    fieldnames = [
        "row_type",
        "timestamp_s",
        "phase",
        "command_linear_mps",
        "command_angular_rps",
        "debug_raw_encoder",
        "odom_x_m",
        "odom_y_m",
        "odom_heading_rad",
        "raw_left_encoder_delta",
        "raw_right_encoder_delta",
        "left_pwm",
        "right_pwm",
        "current_linear_mps",
        "current_angular_rps",
        "packet_hex",
    ]

    log_file = args.out.open("w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(fieldnames)

    print(f"Connected to {args.port} @ {args.baud} baud")
    print(f"Logging to: {args.out}")
    print("Warming up connection...")

    time.sleep(1.0)
    ser.reset_input_buffer()

    def log_command(cmd: Command) -> None:
        packet = encode_command(cmd.linear_mps, cmd.angular_rps, int(args.debug_raw_encoder))
        ser.write(packet)
        write_log_row(
            writer,
            "command",
            timestamp_s=f"{time.time():.6f}",
            phase=cmd.label,
            command_linear_mps=f"{cmd.linear_mps:.6f}",
            command_angular_rps=f"{cmd.angular_rps:.6f}",
            debug_raw_encoder=str(int(args.debug_raw_encoder)),
            odom_x_m="",
            odom_y_m="",
            odom_heading_rad="",
            raw_left_encoder_delta="",
            raw_right_encoder_delta="",
            left_pwm="",
            right_pwm="",
            current_linear_mps="",
            current_angular_rps="",
            packet_hex=packet.hex(),
        )
        log_file.flush()

    try:
        stop_cmd = Command("stop", 0.0, 0.0)
        log_command(stop_cmd)
        time.sleep(0.5)

        for cmd in commands:
            print(f"Running {cmd.label} for {args.duration:.1f}s")
            log_command(cmd)

            phase_end = time.monotonic() + args.duration
            while time.monotonic() < phase_end:
                raw = ser.read(TELEMETRY_PACKET.size)
                if len(raw) != TELEMETRY_PACKET.size:
                    continue

                (
                    odom_x_m,
                    odom_y_m,
                    odom_heading_rad,
                    raw_left_encoder_delta,
                    raw_right_encoder_delta,
                    left_pwm,
                    right_pwm,
                    current_linear_mps,
                    current_angular_rps,
                ) = TELEMETRY_PACKET.unpack(raw)
                write_log_row(
                    writer,
                    "telemetry",
                    timestamp_s=f"{time.time():.6f}",
                    phase=cmd.label,
                    command_linear_mps=f"{cmd.linear_mps:.6f}",
                    command_angular_rps=f"{cmd.angular_rps:.6f}",
                    debug_raw_encoder=str(int(args.debug_raw_encoder)),
                    odom_x_m=f"{odom_x_m:.6f}",
                    odom_y_m=f"{odom_y_m:.6f}",
                    odom_heading_rad=f"{odom_heading_rad:.6f}",
                    raw_left_encoder_delta=str(raw_left_encoder_delta),
                    raw_right_encoder_delta=str(raw_right_encoder_delta),
                    left_pwm=str(left_pwm),
                    right_pwm=str(right_pwm),
                    current_linear_mps=f"{current_linear_mps:.6f}",
                    current_angular_rps=f"{current_angular_rps:.6f}",
                    packet_hex=raw.hex(),
                )
            log_file.flush()

        log_command(stop_cmd)
        time.sleep(0.5)
        return 0
    except KeyboardInterrupt:
        log_command(Command("stop_interrupt", 0.0, 0.0))
        print("\nStopped.")
        return 0
    finally:
        ser.close()
        log_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
