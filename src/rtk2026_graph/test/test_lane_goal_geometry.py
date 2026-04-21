"""Тесты смещения цели в полосу (совпадают с lane_decision_manager)."""

import math

import pytest

from rtk2026_graph.lane_goal_geometry import clamp, project_goal_on_lane


def test_clamp_lane_offset_to_half_width_minus_margin() -> None:
    gx, gy, yaw, co = project_goal_on_lane(
        10.0,
        0.0,
        ((8.0, 0.0), (10.0, 0.0)),
        lane_right_offset_m=1.0,
        lane_half_width_m=0.25,
        lane_safety_margin_m=0.03,
    )
    assert co == pytest.approx(0.22)
    assert gx == pytest.approx(10.0)
    assert gy == pytest.approx(-0.22)


def test_lane_goal_offset_sign_minus_one_mirrors_across_centerline() -> None:
    gx_pos, gy_pos, _, _ = project_goal_on_lane(
        10.0,
        0.0,
        ((8.0, 0.0), (10.0, 0.0)),
        lane_right_offset_m=0.2,
        lane_half_width_m=0.25,
        lane_safety_margin_m=0.03,
        lane_goal_offset_sign=1,
    )
    gx_neg, gy_neg, _, _ = project_goal_on_lane(
        10.0,
        0.0,
        ((8.0, 0.0), (10.0, 0.0)),
        lane_right_offset_m=0.2,
        lane_half_width_m=0.25,
        lane_safety_margin_m=0.03,
        lane_goal_offset_sign=-1,
    )
    assert gy_pos == pytest.approx(-gy_neg)
    assert gx_pos == pytest.approx(gx_neg)


def test_right_offset_along_eastbound_segment() -> None:
    """Последний сегмент вдоль +X: «вправо» по полосе — в сторону -Y."""
    gx, gy, yaw, _ = project_goal_on_lane(
        10.0,
        0.0,
        ((8.0, 0.0), (10.0, 0.0)),
        lane_right_offset_m=0.2,
        lane_half_width_m=0.25,
        lane_safety_margin_m=0.03,
    )
    assert yaw == pytest.approx(0.0)
    assert gx == pytest.approx(10.0)
    assert gy == pytest.approx(-0.2)


def test_short_polyline_keeps_vertex_yaw_zero() -> None:
    gx, gy, yaw, co = project_goal_on_lane(
        5.0,
        3.0,
        ((5.0, 3.0),),
        lane_right_offset_m=0.2,
        lane_half_width_m=0.25,
        lane_safety_margin_m=0.03,
    )
    assert gx == pytest.approx(5.0)
    assert gy == pytest.approx(3.0)
    assert yaw == pytest.approx(0.0)
    assert co == pytest.approx(0.2)


def test_clamp_helper() -> None:
    assert clamp(5.0, 0.0, 10.0) == 5.0
    assert clamp(-1.0, 0.0, 10.0) == 0.0
    assert clamp(11.0, 0.0, 10.0) == 10.0


def test_northbound_yaw_pi_half() -> None:
    gx, gy, yaw, _ = project_goal_on_lane(
        0.0,
        10.0,
        ((0.0, 8.0), (0.0, 10.0)),
        lane_right_offset_m=0.1,
        lane_half_width_m=0.25,
        lane_safety_margin_m=0.03,
    )
    assert yaw == pytest.approx(math.pi / 2)
    assert gx == pytest.approx(0.1)
    assert gy == pytest.approx(10.0)
