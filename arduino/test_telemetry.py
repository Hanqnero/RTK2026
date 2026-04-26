#!/usr/bin/env python3
"""Simple telemetry reader for the Arduino firmware.

Reads fixed-size binary packets from serial and prints decoded IMU + odometry.
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
# uint8 imu_online
# uint8 imu_chip_id
# int16 imu_acc_x, imu_acc_y, imu_acc_z
# int16 imu_gyro_x, imu_gyro_y, imu_gyro_z
# float odom_x_m, odom_y_m, odom_heading_rad
PACKET_STRUCT = struct.Struct("<BBhhhhhhfff")
PACKET_SIZE = PACKET_STRUCT.size
EXPECTED_CHIP_IDS = {0x24}


def parse_args() -> argparse.Namespace:
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


def format_packet(fields: tuple[int, ... | float]) -> str:
    (
        imu_online,
        imu_chip_id,
        imu_acc_x,
        imu_acc_y,
        imu_acc_z,
        imu_gyro_x,
        imu_gyro_y,
        imu_gyro_z,
        odom_x_m,
        odom_y_m,
        odom_heading_rad,
    ) = fields

    heading_deg = odom_heading_rad * 180.0 / math.pi
    chip_str = f"0x{imu_chip_id:02X}"
    status = "OK" if imu_online and imu_chip_id in EXPECTED_CHIP_IDS else "WARN"

    return (
        f"[{status}] "
        f"IMU online={imu_online} chip={chip_str} "
        f"acc=({imu_acc_x:6d},{imu_acc_y:6d},{imu_acc_z:6d}) "
        f"gyro=({imu_gyro_x:6d},{imu_gyro_y:6d},{imu_gyro_z:6d}) "
        f"odom=(x={odom_x_m: .3f} m, y={odom_y_m: .3f} m, yaw={odom_heading_rad: .3f} rad / {heading_deg: .1f} deg)"
    )


def main() -> int:
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
