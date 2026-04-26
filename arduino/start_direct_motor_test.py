#!/usr/bin/env python3
"""Start the direct motor test firmware and read telemetry."""

from __future__ import annotations

import argparse
import csv
import math
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


PACKET_STRUCT = struct.Struct("<fffiihh")
PACKET_SIZE = PACKET_STRUCT.size
START_COMMAND = b"start"


@dataclass
class TelemetrySample:
    timestamp_s: float
    odom_x_m: float
    odom_y_m: float
    odom_heading_rad: float
    raw_left_encoder_delta: int
    raw_right_encoder_delta: int
    left_pwm: int
    right_pwm: int

    @property
    def odom_heading_deg(self) -> float:
        return self.odom_heading_rad * 180.0 / math.pi


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send 'start' to direct motor test firmware and read telemetry"
    )
    parser.add_argument("--port", required=True, help="Serial device path")
    parser.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--hz", type=float, default=10.0, help="Print rate limit in Hz (default: 10)"
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=1.0,
        help="Seconds to wait after opening serial before sending start (default: 1.0)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        help="Optional CSV log file path",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw telemetry packet bytes in hex",
    )
    return parser.parse_args()


def format_sample(sample: TelemetrySample) -> str:
    return (
        f"t={sample.timestamp_s:10.3f}s "
        f"odom=(x={sample.odom_x_m: .3f} m, y={sample.odom_y_m: .3f} m, "
        f"yaw={sample.odom_heading_rad: .3f} rad / {sample.odom_heading_deg: .1f} deg) "
        f"enc=(L={sample.raw_left_encoder_delta:+d}, R={sample.raw_right_encoder_delta:+d}) "
        f"pwm=(L={sample.left_pwm:+d}, R={sample.right_pwm:+d})"
    )


def parse_sample(raw: bytes) -> TelemetrySample:
    (
        odom_x_m,
        odom_y_m,
        odom_heading_rad,
        raw_left_encoder_delta,
        raw_right_encoder_delta,
        left_pwm,
        right_pwm,
    ) = PACKET_STRUCT.unpack(raw)

    return TelemetrySample(
        timestamp_s=time.time(),
        odom_x_m=odom_x_m,
        odom_y_m=odom_y_m,
        odom_heading_rad=odom_heading_rad,
        raw_left_encoder_delta=raw_left_encoder_delta,
        raw_right_encoder_delta=raw_right_encoder_delta,
        left_pwm=left_pwm,
        right_pwm=right_pwm,
    )


def write_csv_header(writer: csv.writer) -> None:
    writer.writerow(
        [
            "timestamp_s",
            "odom_x_m",
            "odom_y_m",
            "odom_heading_rad",
            "odom_heading_deg",
            "raw_left_encoder_delta",
            "raw_right_encoder_delta",
            "left_pwm",
            "right_pwm",
        ]
    )


def write_csv_sample(writer: csv.writer, sample: TelemetrySample) -> None:
    writer.writerow(
        [
            f"{sample.timestamp_s:.6f}",
            f"{sample.odom_x_m:.6f}",
            f"{sample.odom_y_m:.6f}",
            f"{sample.odom_heading_rad:.6f}",
            f"{sample.odom_heading_deg:.6f}",
            f"{sample.raw_left_encoder_delta:d}",
            f"{sample.raw_right_encoder_delta:d}",
            f"{sample.left_pwm:d}",
            f"{sample.right_pwm:d}",
        ]
    )


def main() -> int:
    args = parse_args()
    min_print_period_s = 1.0 / max(args.hz, 0.1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    log_file = None
    csv_writer = None
    if args.out:
        log_file = args.out.open("w", newline="")
        csv_writer = csv.writer(log_file)
        write_csv_header(csv_writer)

    try:
        print(f"Connected to {args.port} @ {args.baud} baud")
        print(f"Packet size: {PACKET_SIZE} bytes")
        if args.out:
            print(f"Logging to: {args.out}")

        time.sleep(args.startup_delay)
        ser.reset_input_buffer()
        ser.write(START_COMMAND)
        ser.flush()
        print("Sent start command")

        last_print = 0.0
        while True:
            raw = ser.read(PACKET_SIZE)
            if len(raw) != PACKET_SIZE:
                continue

            if args.raw:
                print(raw.hex())

            sample = parse_sample(raw)

            if csv_writer is not None:
                write_csv_sample(csv_writer, sample)
                if log_file is not None:
                    log_file.flush()

            now = time.monotonic()
            if now - last_print >= min_print_period_s:
                print(format_sample(sample))
                last_print = now
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        ser.close()
        if log_file is not None:
            log_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
