import struct
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
DRIVER_SRC = REPO_ROOT / "src" / "rtk2026_driver"

if str(DRIVER_SRC) not in sys.path:
    sys.path.insert(0, str(DRIVER_SRC))


from rtk2026_driver.protocol import (
    COMMAND_STRUCT,
    TELEMETRY_STRUCT,
    pack_command,
    pop_telemetry_packet,
)


def _make_telemetry_bytes(
    odom_x_m=1.25,
    odom_y_m=-0.50,
    odom_heading_rad=0.75,
    raw_left_encoder_delta=120,
    raw_right_encoder_delta=-95,
    left_pwm=150,
    right_pwm=-170,
    current_linear_mps=0.42,
    current_angular_rps=-0.18,
):
    return struct.pack(
        "<fffiihhff",
        odom_x_m,
        odom_y_m,
        odom_heading_rad,
        raw_left_encoder_delta,
        raw_right_encoder_delta,
        left_pwm,
        right_pwm,
        current_linear_mps,
        current_angular_rps,
    )


def test_protocol_struct_sizes_match_arduino():
    assert COMMAND_STRUCT.size == 9
    assert TELEMETRY_STRUCT.size == 32


def test_pack_command_creates_expected_binary_packet():
    payload = pack_command(
        linear_mps=0.25,
        angular_rps=-0.50,
        debug_raw_encoder=True,
    )

    assert len(payload) == 9

    linear_mps, angular_rps, debug_flag = struct.unpack(
        "<ffB",
        payload,
    )

    assert linear_mps == pytest.approx(0.25)
    assert angular_rps == pytest.approx(-0.50)
    assert debug_flag == 1


def test_pack_command_writes_zero_debug_flag():
    payload = pack_command(
        linear_mps=0.0,
        angular_rps=0.0,
        debug_raw_encoder=False,
    )

    _, _, debug_flag = struct.unpack(
        "<ffB",
        payload,
    )

    assert debug_flag == 0


def test_pop_telemetry_returns_none_for_incomplete_packet():
    complete_packet = _make_telemetry_bytes()

    buffer = bytearray(
        complete_packet[:15]
    )

    original_buffer = bytes(buffer)

    result = pop_telemetry_packet(buffer)

    assert result is None
    assert bytes(buffer) == original_buffer


def test_pop_telemetry_decodes_complete_packet():
    buffer = bytearray(
        _make_telemetry_bytes()
    )

    packet = pop_telemetry_packet(buffer)

    assert packet is not None

    assert packet.odom_x_m == pytest.approx(1.25)
    assert packet.odom_y_m == pytest.approx(-0.50)

    assert (
        packet.odom_heading_rad
        == pytest.approx(0.75)
    )

    assert packet.raw_left_encoder_delta == 120
    assert packet.raw_right_encoder_delta == -95

    assert packet.left_pwm == 150
    assert packet.right_pwm == -170

    assert (
        packet.current_linear_mps
        == pytest.approx(0.42)
    )

    assert (
        packet.current_angular_rps
        == pytest.approx(-0.18)
    )

    assert buffer == bytearray()


def test_pop_telemetry_removes_only_one_packet():
    first_raw = _make_telemetry_bytes(
        odom_x_m=1.0,
        raw_left_encoder_delta=10,
    )

    second_raw = _make_telemetry_bytes(
        odom_x_m=2.0,
        raw_left_encoder_delta=20,
    )

    buffer = bytearray(
        first_raw + second_raw
    )

    first_packet = pop_telemetry_packet(buffer)

    assert first_packet is not None
    assert first_packet.odom_x_m == pytest.approx(1.0)
    assert first_packet.raw_left_encoder_delta == 10

    assert len(buffer) == TELEMETRY_STRUCT.size

    second_packet = pop_telemetry_packet(buffer)

    assert second_packet is not None
    assert second_packet.odom_x_m == pytest.approx(2.0)
    assert second_packet.raw_left_encoder_delta == 20

    assert buffer == bytearray()


def test_pop_telemetry_keeps_incomplete_next_packet():
    first_raw = _make_telemetry_bytes(
        odom_x_m=1.0,
    )

    second_raw = _make_telemetry_bytes(
        odom_x_m=2.0,
    )

    second_part = second_raw[:7]

    buffer = bytearray(
        first_raw + second_part
    )

    first_packet = pop_telemetry_packet(buffer)

    assert first_packet is not None
    assert first_packet.odom_x_m == pytest.approx(1.0)

    assert bytes(buffer) == second_part

    second_packet = pop_telemetry_packet(buffer)

    assert second_packet is None
    assert bytes(buffer) == second_part


def test_telemetry_supports_int16_pwm_boundaries():
    buffer = bytearray(
        _make_telemetry_bytes(
            left_pwm=-32768,
            right_pwm=32767,
        )
    )

    packet = pop_telemetry_packet(buffer)

    assert packet is not None
    assert packet.left_pwm == -32768
    assert packet.right_pwm == 32767


def test_telemetry_supports_signed_encoder_deltas():
    buffer = bytearray(
        _make_telemetry_bytes(
            raw_left_encoder_delta=-2_000_000,
            raw_right_encoder_delta=2_000_000,
        )
    )

    packet = pop_telemetry_packet(buffer)

    assert packet is not None

    assert (
        packet.raw_left_encoder_delta
        == -2_000_000
    )

    assert (
        packet.raw_right_encoder_delta
        == 2_000_000
    )