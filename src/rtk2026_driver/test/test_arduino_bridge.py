# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import struct

import pytest

from rtk2026_driver.arduino_bridge_node import clamp_pwm


def test_clamp_pwm():
    assert clamp_pwm(0) == 0
    assert clamp_pwm(100) == 100
    assert clamp_pwm(127) == 127
    assert clamp_pwm(200) == 127
    assert clamp_pwm(-50) == -50
    assert clamp_pwm(-128) == -128
    assert clamp_pwm(-200) == -128


def test_parse_encoder_32_bytes():
    """Protocol: left_speed(4), left_cnt(4), right_speed(4), right_cnt(4) little-endian."""
    left_speed = 10
    left_cnt = -100
    right_speed = -5
    right_cnt = 200
    raw = struct.pack("<iiii", left_speed, left_cnt, right_speed, right_cnt) + bytes(16)
    assert len(raw) == 32
    a, b, c, d = struct.unpack("<iiii", raw[0:16])
    assert a == left_speed
    assert b == left_cnt
    assert c == right_speed
    assert d == right_cnt
