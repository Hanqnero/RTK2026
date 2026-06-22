#!/usr/bin/env python3
"""Verify wheel command, encoder, and PWM signs through the live firmware."""

from __future__ import annotations

import argparse
import struct
import sys
import time

CONTROL_PACKET = struct.Struct("<ffB")
TELEMETRY_PACKET = struct.Struct("<fffiihh")
TRACK_WIDTH_M = 0.040


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check that positive wheel commands produce positive encoder deltas"
    )
    parser.add_argument("--port", required=True, help="Serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument(
        "--wheel-mps",
        type=float,
        default=0.08,
        help="Single-wheel test speed in m/s (default: 0.08)",
    )
    parser.add_argument(
        "--pulse",
        type=float,
        default=1.0,
        help="Seconds to hold each test command (default: 1.0)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=0.5,
        help="Seconds to send zero command between tests (default: 0.5)",
    )
    return parser.parse_args()


def command_for_wheels(left_mps: float, right_mps: float) -> bytes:
    linear_mps = 0.5 * (left_mps + right_mps)
    angular_rps = (right_mps - left_mps) / TRACK_WIDTH_M
    return CONTROL_PACKET.pack(linear_mps, angular_rps, 1)


def write_wheels(ser: serial.Serial, left_mps: float, right_mps: float) -> None:
    ser.write(command_for_wheels(left_mps, right_mps))
    ser.flush()


def read_packets_until(
    ser: serial.Serial, deadline: float
) -> list[tuple[float, float, float, int, int, int, int]]:
    packets: list[tuple[float, float, float, int, int, int, int]] = []
    while time.monotonic() < deadline:
        raw = ser.read(TELEMETRY_PACKET.size)
        if len(raw) == TELEMETRY_PACKET.size:
            packets.append(TELEMETRY_PACKET.unpack(raw))
    return packets


def send_stop(ser: serial.Serial, seconds: float) -> None:
    deadline = time.monotonic() + max(seconds, 0.0)
    while time.monotonic() < deadline:
        write_wheels(ser, 0.0, 0.0)
        read_packets_until(ser, min(time.monotonic() + 0.05, deadline))


def sign(value: int) -> int:
    return (value > 0) - (value < 0)


def run_case(
    ser: serial.Serial,
    wheel: str,
    direction: int,
    wheel_mps: float,
    pulse_s: float,
) -> bool:
    left_cmd = direction * wheel_mps if wheel == "left" else 0.0
    right_cmd = direction * wheel_mps if wheel == "right" else 0.0

    ser.reset_input_buffer()
    deadline = time.monotonic() + pulse_s
    packets = []
    while time.monotonic() < deadline:
        write_wheels(ser, left_cmd, right_cmd)
        packets.extend(read_packets_until(ser, min(time.monotonic() + 0.05, deadline)))

    left_sum = sum(packet[3] for packet in packets)
    right_sum = sum(packet[4] for packet in packets)
    left_pwm = packets[-1][5] if packets else 0
    right_pwm = packets[-1][6] if packets else 0

    encoder_sum = left_sum if wheel == "left" else right_sum
    other_sum = right_sum if wheel == "left" else left_sum
    pwm = left_pwm if wheel == "left" else right_pwm
    ok = sign(encoder_sum) == direction and abs(encoder_sum) >= max(2, abs(other_sum))
    status = "PASS" if ok else f"FAIL flip k{wheel.title()}EncoderReverse"

    print(
        f"{wheel:5s} {direction:+d}: enc={encoder_sum:+d} other={other_sum:+d} "
        f"pwm={pwm:+d} {status}"
    )
    return ok


def main() -> int:
    args = parse_args()

    try:
        import serial
    except ImportError as exc:  # pragma: no cover
        raise SystemExit("pyserial is required. Install with: pip install pyserial") from exc

    speed = abs(args.wheel_mps)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    try:
        print("Lift the robot before running this test.")
        time.sleep(1.0)
        send_stop(ser, args.settle)

        results = [
            run_case(ser, "left", +1, speed, args.pulse),
            run_case(ser, "left", -1, speed, args.pulse),
            run_case(ser, "right", +1, speed, args.pulse),
            run_case(ser, "right", -1, speed, args.pulse),
        ]
        return 0 if all(results) else 2
    finally:
        send_stop(ser, args.settle)
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
