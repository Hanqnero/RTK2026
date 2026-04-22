import struct
import math

from rtk2026_driver.protocol import (
    CMD_HEADER,
    CMD_SIZE,
    TEL_HEADER,
    InvalidChecksumError,
    pack_command,
    parse_telemetry,
)


def test_pack_command_builds_framed_linear_angular_packet():
    packet = pack_command(0.12, -0.34)

    assert len(packet) == CMD_SIZE == 11
    assert packet[:2] == CMD_HEADER

    linear, angular = struct.unpack("<ff", packet[2:10])
    assert math.isclose(linear, 0.12, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(angular, -0.34, rel_tol=1e-6, abs_tol=1e-6)

    expected_checksum = sum(packet[:-1]) & 0xFF
    assert packet[-1] == expected_checksum


def test_parse_telemetry_returns_counts_and_consumes_buffer():
    left_count = 123456
    right_count = -654321
    payload = TEL_HEADER + struct.pack("<ii", left_count, right_count)
    frame = payload + bytes([sum(payload) & 0xFF])
    buf = bytearray(frame)

    parsed = parse_telemetry(buf)

    assert parsed is not None
    assert parsed.left_count == left_count
    assert parsed.right_count == right_count
    assert buf == bytearray()


def test_parse_telemetry_keeps_partial_frame():
    payload = TEL_HEADER + struct.pack("<ii", 1, 2)
    frame = payload + bytes([sum(payload) & 0xFF])
    partial = bytearray(frame[:-2])

    parsed = parse_telemetry(partial)

    assert parsed is None
    assert partial == bytearray(frame[:-2])


def test_parse_telemetry_raises_on_bad_checksum():
    payload = TEL_HEADER + struct.pack("<ii", 10, 20)
    frame = bytearray(payload + bytes([0x00]))

    try:
        parse_telemetry(frame)
    except InvalidChecksumError:
        pass
    else:
        raise AssertionError("InvalidChecksumError was not raised")
