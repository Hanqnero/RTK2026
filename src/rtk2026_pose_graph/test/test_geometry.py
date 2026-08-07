import math

import pytest

from rtk2026_pose_graph.geometry import (
    interpolate_along_polyline,
    point_to_polyline_distance_m,
    polyline_length_m,
    polyline_signed_lateral_m,
    project_point_on_polyline,
)

_HORIZONTAL = ((0.0, 0.0), (1.0, 0.0))
_L_SHAPE = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))


def test_signed_lateral_positive_on_left() -> None:
    # Правило правой руки: точка выше сегмента (0,0)->(1,0) лежит слева.
    assert polyline_signed_lateral_m(0.5, 1.0, _HORIZONTAL) == 1.0


def test_signed_lateral_negative_on_right() -> None:
    assert polyline_signed_lateral_m(0.5, -1.0, _HORIZONTAL) == -1.0


def test_signed_lateral_picks_nearest_segment() -> None:
    # (1.5, 0.5) однозначно ближе ко второму сегменту (1,0)->(1,1)
    # (dist^2 0.25 против 0.5 до первого); справа от него - x > 1.
    assert polyline_signed_lateral_m(1.5, 0.5, _L_SHAPE) == -0.5


def test_degenerate_polyline_has_no_side() -> None:
    assert polyline_signed_lateral_m(1.0, 1.0, ()) == 0.0
    assert polyline_signed_lateral_m(1.0, 1.0, ((0.0, 0.0),)) == 0.0
    assert point_to_polyline_distance_m(1.0, 1.0, ()) == 0.0
    assert project_point_on_polyline(1.0, 1.0, ()) is None


def test_distance_is_unsigned_and_clamped_to_segment_ends() -> None:
    # Точка за концом сегмента: расстояние считается до самого конца,
    # а не до бесконечной прямой.
    assert point_to_polyline_distance_m(3.0, 0.0, _HORIZONTAL) == 2.0
    assert point_to_polyline_distance_m(0.5, -1.0, _HORIZONTAL) == 1.0


def test_projection_reports_point_arc_length_and_heading() -> None:
    projection = project_point_on_polyline(0.25, 0.5, _HORIZONTAL)

    assert projection is not None
    assert (projection.x, projection.y) == (0.25, 0.0)
    assert projection.distance_m == 0.5
    assert projection.signed_lateral_m == 0.5
    assert projection.arc_length_m == 0.25
    assert projection.segment_index == 0
    assert projection.heading_rad == 0.0


def test_projection_arc_length_accumulates_across_segments() -> None:
    projection = project_point_on_polyline(1.2, 0.5, _L_SHAPE)

    assert projection is not None
    assert projection.segment_index == 1
    # Первый сегмент длиной 1.0 плюс 0.5 по второму.
    assert projection.arc_length_m == pytest.approx(1.5)
    assert projection.heading_rad == pytest.approx(math.pi / 2.0)


def test_projection_skips_degenerate_segments() -> None:
    # Повторяющаяся точка не даёт направления, поэтому как кандидат
    # пропускается, но длину дуги не ломает.
    polyline = ((0.0, 0.0), (0.0, 0.0), (1.0, 0.0))

    projection = project_point_on_polyline(0.5, 0.5, polyline)

    assert projection is not None
    assert projection.segment_index == 1
    assert projection.arc_length_m == pytest.approx(0.5)


def test_polyline_length() -> None:
    assert polyline_length_m(_HORIZONTAL) == 1.0
    assert polyline_length_m(_L_SHAPE) == 2.0
    assert polyline_length_m(((0.0, 0.0), (3.0, 4.0))) == 5.0
    assert polyline_length_m(()) == 0.0
    assert polyline_length_m(((1.0, 1.0),)) == 0.0


def test_interpolate_along_polyline_inside_first_segment() -> None:
    point = interpolate_along_polyline(_L_SHAPE, 0.25)

    assert point is not None
    x, y, heading = point
    assert (x, y) == (0.25, 0.0)
    assert heading == 0.0


def test_interpolate_along_polyline_crosses_into_second_segment() -> None:
    point = interpolate_along_polyline(_L_SHAPE, 1.5)

    assert point is not None
    x, y, heading = point
    assert (x, y) == pytest.approx((1.0, 0.5))
    assert heading == pytest.approx(math.pi / 2.0)


def test_interpolate_clamps_outside_range() -> None:
    # За пределами полилинии возвращается её конец, а не None: вызывающий
    # всегда получает точку на пути.
    beyond = interpolate_along_polyline(_L_SHAPE, 99.0)
    assert beyond is not None
    assert (beyond[0], beyond[1]) == (1.0, 1.0)

    before = interpolate_along_polyline(_L_SHAPE, -5.0)
    assert before is not None
    assert (before[0], before[1]) == (0.0, 0.0)


def test_interpolate_needs_two_points() -> None:
    assert interpolate_along_polyline((), 1.0) is None
    assert interpolate_along_polyline(((0.0, 0.0),), 1.0) is None
