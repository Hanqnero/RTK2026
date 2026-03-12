# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import math

import pytest


def test_diff_drive_kinematics():
    """v_l = v - omega*L/2, v_r = v + omega*L/2."""
    L = 0.3
    v = 0.2
    omega = 0.5
    v_l = v - omega * L / 2.0
    v_r = v + omega * L / 2.0
    assert abs(v_l - (0.2 - 0.075)) < 1e-9
    assert abs(v_r - (0.2 + 0.075)) < 1e-9


def test_odometry_delta():
    """d_center = (d_left + d_right)/2, d_theta = (d_right - d_left)/L."""
    L = 0.3
    meters_per_tick = 0.001
    d_left_ticks = 100
    d_right_ticks = 120
    d_left = d_left_ticks * meters_per_tick
    d_right = d_right_ticks * meters_per_tick
    d_center = (d_left + d_right) / 2.0
    d_theta = (d_right - d_left) / L
    assert abs(d_center - 0.11) < 1e-9
    assert abs(d_theta - (0.02 / 0.3)) < 1e-9


def test_pose_update():
    """dx = d_center*cos(theta), dy = d_center*sin(theta)."""
    theta = math.pi / 4
    d_center = 0.1
    dx = d_center * math.cos(theta)
    dy = d_center * math.sin(theta)
    expected = 0.1 * (math.sqrt(2) / 2)
    assert abs(dx - expected) < 1e-9
    assert abs(dy - expected) < 1e-9
