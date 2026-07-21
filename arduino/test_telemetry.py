#!/usr/bin/env python3
"""Минимальный reader телеметрии основной Arduino firmware.

Reads fixed-size binary packets from serial and prints decoded odometry, encoder debug values, and PWM commands.
Packet layout matches TelemetryPacket in include/control_protocol.h.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install with: pip install pyserial"
    ) from exc


# Little-endian packed layout:
# float odom_x_m, odom_y_m, odom_heading_rad
# int32 raw_left_encoder_delta, raw_right_encoder_delta
# int16 left_pwm, right_pwm
# float current_linear_mps, current_angular_rps
PACKET_STRUCT = struct.Struct("<fffiihhff")
PACKET_SIZE = PACKET_STRUCT.size


def parse_args() -> argparse.Namespace:
    """Разобрать Serial-порт и ограничение частоты печати."""

    parser = argparse.ArgumentParser(description="Read Arduino telemetry packets")
    parser.add_argument(
        "--port", required=True, help="Serial device, e.g. /dev/cu.usbserial-110"
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--hz", type=float, default=10.0, help="Print rate limit in Hz (default: 10)"
    )
    parser.add_argument(
        "--raw", action="store_true", help="Print raw packet bytes in hex"
    )
    return parser.parse_args()


def format_packet(fields: tuple[float, float, float, int, int, int, int, float, float]) -> str:
    """Преобразовать распакованный ``TelemetryPacket`` в одну строку."""

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
    ) = fields

    heading_deg = odom_heading_rad * 180.0 / math.pi

    return (
        f"odom=(x={odom_x_m: .3f} m, y={odom_y_m: .3f} m, yaw={odom_heading_rad: .3f} rad / {heading_deg: .1f} deg) "
        f"speed=(linear={current_linear_mps:+.3f} m/s, angular={current_angular_rps:+.3f} rad/s) "
        f"enc=(L={raw_left_encoder_delta:+d}, R={raw_right_encoder_delta:+d}) "
        f"pwm=(L={left_pwm:+d}, R={right_pwm:+d})"
    )


def main() -> int:
    """Читать фиксированные записи и печатать их с заданной частотой."""

    args = parse_args()
    min_period = 1.0 / max(args.hz, 0.1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Connected to {args.port} @ {args.baud} baud")
    print(f"Packet size: {PACKET_SIZE} bytes")

    # Give the board a moment if opening serial triggers reset.
    time.sleep(1.0)
    ser.reset_input_buffer()

    last_print = 0.0
    try:
        while True:
            raw = ser.read(PACKET_SIZE)
            if len(raw) != PACKET_SIZE:
                continue

            if args.raw:
                print(raw.hex())

            fields = PACKET_STRUCT.unpack(raw)

            now = time.monotonic()
            if now - last_print >= min_period:
                print(format_packet(fields))
                last_print = now
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
