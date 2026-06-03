#!/usr/bin/env python3
"""Read the BMI270 test firmware serial console.

Examples:
  python3 read_bmi270_console.py --port /dev/cu.usbmodem1101
  python3 read_bmi270_console.py --port COM3 --timestamp
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install with: pip install pyserial"
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read BMI270 test firmware output")
    parser.add_argument("--port", required=True, help="Serial device path")
    parser.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Serial read timeout in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--timestamp",
        action="store_true",
        help="Prefix each received line with host time",
    )
    parser.add_argument(
        "--no-reset-buffer",
        action="store_true",
        help="Do not clear buffered serial data after opening the port",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Connected to {args.port} @ {args.baud} baud")
    print("Press Ctrl+C to stop.")

    time.sleep(1.0)
    if not args.no_reset_buffer:
        ser.reset_input_buffer()

    try:
        while True:
            raw_line = ser.readline()
            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="replace").rstrip("\r\n")
            if args.timestamp:
                print(f"{time.time():.3f} {line}")
            else:
                print(line)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
