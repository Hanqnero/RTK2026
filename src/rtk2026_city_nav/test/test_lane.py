import math

import pytest

from rtk2026_city_nav.lane import (
    LEFT_HAND_TRAFFIC,
    offset_along_chain,
    resample_along_chain,
    right_normal,
)

#: Прямая на восток, длина 2 м.
EAST = ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0))

#: Излом на 90 градусов: на восток, затем на север.
CORNER = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))


def test_right_normal_is_south_when_heading_east() -> None:
    # Лицом на восток правая рука смотрит на юг: y уменьшается.
    assert right_normal((1.0, 0.0)) == (0.0, -1.0)


def test_right_normal_is_east_when_heading_north() -> None:
    assert right_normal((0.0, 1.0)) == (1.0, 0.0)


def test_offset_puts_lane_right_of_travel() -> None:
    poses = offset_along_chain(EAST, lane_offset_m=0.2)

    assert [p.xy for p in poses] == [(0.0, -0.2), (1.0, -0.2), (2.0, -0.2)]
    assert all(p.yaw == 0.0 for p in poses)


def test_reversed_order_puts_lane_on_the_other_side() -> None:
    forward = offset_along_chain(EAST, lane_offset_m=0.2)
    backward = offset_along_chain(tuple(reversed(EAST)), lane_offset_m=0.2)

    # Та же линия, обратный порядок пары - физически противоположная сторона.
    assert forward[0].y == -0.2
    assert backward[0].y == +0.2


def test_left_hand_traffic_flips_the_side() -> None:
    poses = offset_along_chain(
        EAST, lane_offset_m=0.2, traffic_side=LEFT_HAND_TRAFFIC
    )

    assert all(p.y == pytest.approx(+0.2) for p in poses)


def test_bend_pose_is_equidistant_from_both_lines() -> None:
    offset = 0.2
    poses = offset_along_chain(CORNER, lane_offset_m=offset)

    # Три позы: начало, излом, конец. Излом на 90 градусов, вынос умеренный.
    assert len(poses) == 3
    corner = poses[1]

    # Первая прямая: y = 0, движение на восток, полоса южнее.
    # Вторая прямая: x = 1, движение на север, полоса восточнее.
    assert abs(corner.y - 0.0) == pytest.approx(offset)
    assert abs(corner.x - 1.0) == pytest.approx(offset)


def test_bend_miter_grows_with_sharper_angle() -> None:
    shallow = ((0.0, 0.0), (1.0, 0.0), (2.0, 0.2))
    sharp = ((0.0, 0.0), (1.0, 0.0), (1.2, 0.9))

    def miter(polyline: tuple[tuple[float, float], ...]) -> float:
        poses = offset_along_chain(polyline, lane_offset_m=0.2, miter_limit=99.0)
        corner = poses[1]
        return math.hypot(corner.x - 1.0, corner.y - 0.0)

    assert miter(sharp) > miter(shallow)


def test_bend_miter_diverges_near_uturn() -> None:
    # Излом почти на 180 градусов: вынос по биссектрисе уходит в бесконечность.
    uturn = ((0.0, 0.0), (1.0, 0.0), (0.0, 0.02))

    # Предел заведомо выше, чем нужно, чтобы увидеть саму вырожденность.
    poses = offset_along_chain(uturn, lane_offset_m=0.2, miter_limit=1000.0)

    assert len(poses) == 3
    corner = poses[1]
    # Смещение 0.2 м, а поза улетела на десятки метров.
    assert math.hypot(corner.x - 1.0, corner.y - 0.0) > 10.0


def test_miter_limit_rounds_the_corner_with_two_poses() -> None:
    uturn = ((0.0, 0.0), (1.0, 0.0), (0.0, 0.02))

    poses = offset_along_chain(uturn, lane_offset_m=0.2, miter_limit=2.0)

    # Вместо одной вынесенной позы - две, по обе стороны излома.
    # Крайние позы - это концы сегментов, они в метре отсюда и не при чём.
    assert len(poses) == 4

    for corner_pose in (poses[1], poses[2]):
        distance = math.hypot(corner_pose.x - 1.0, corner_pose.y - 0.0)
        # Скруглённая поза отстоит от излома ровно на смещение полосы.
        assert distance == pytest.approx(0.2)


def test_degenerate_polyline_gives_no_poses() -> None:
    assert offset_along_chain((), lane_offset_m=0.2) == ()
    assert offset_along_chain(((1.0, 1.0),), lane_offset_m=0.2) == ()
    # Повторяющаяся точка не задаёт направления.
    assert offset_along_chain(((1.0, 1.0), (1.0, 1.0)), lane_offset_m=0.2) == ()


def test_repeated_points_inside_polyline_are_skipped() -> None:
    poles = ((0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (2.0, 0.0))

    poses = offset_along_chain(poles, lane_offset_m=0.2)

    assert all(p.y == pytest.approx(-0.2) for p in poses)
    assert all(p.yaw == 0.0 for p in poses)


def test_resample_keeps_original_poses_and_respects_step() -> None:
    poses = resample_along_chain(EAST, lane_offset_m=0.2, step_m=0.5)

    xs = [p.x for p in poses]
    # Исходные точки на месте.
    for original in (0.0, 1.0, 2.0):
        assert any(x == pytest.approx(original) for x in xs)

    # Шаг не превышен.
    for current, following in zip(poses, poses[1:]):
        span = math.hypot(following.x - current.x, following.y - current.y)
        assert span <= 0.5 + 1e-9


def test_resample_without_step_returns_base_poses() -> None:
    base = offset_along_chain(EAST, lane_offset_m=0.2)

    assert resample_along_chain(EAST, lane_offset_m=0.2, step_m=0.0) == base


def test_yaw_follows_direction_of_travel() -> None:
    poses = offset_along_chain(CORNER, lane_offset_m=0.2)

    assert poses[0].yaw == pytest.approx(0.0)
    assert poses[-1].yaw == pytest.approx(math.pi / 2.0)
