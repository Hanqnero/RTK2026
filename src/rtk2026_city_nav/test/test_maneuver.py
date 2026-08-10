import math

import pytest

from rtk2026_city_nav.maneuver import (
    MANEUVER_ORDER,
    Maneuver,
    classify,
    classify_candidates,
    tangent_at_end,
    chord_direction,
    turn_angle,
)

EAST = (1.0, 0.0)
NORTH = (0.0, 1.0)
WEST = (-1.0, 0.0)
SOUTH = (0.0, -1.0)

TOL = math.radians(30.0)


def test_turn_angle_sign_matches_right_hand_rule() -> None:
    # Ехал на север, поворачиваю на восток - это направо, угол отрицательный.
    assert turn_angle(NORTH, EAST) == pytest.approx(-math.pi / 2.0)
    # На север, затем на запад - налево.
    assert turn_angle(NORTH, WEST) == pytest.approx(+math.pi / 2.0)


def test_turn_angle_zero_when_direction_kept() -> None:
    assert turn_angle(EAST, EAST) == pytest.approx(0.0)


def test_turn_angle_is_pi_for_uturn() -> None:
    assert abs(turn_angle(EAST, WEST)) == pytest.approx(math.pi)


def test_classify_covers_four_classes() -> None:
    assert classify(0.0, straight_tolerance_rad=TOL) is Maneuver.STRAIGHT
    assert classify(math.pi, straight_tolerance_rad=TOL) is Maneuver.UTURN
    assert classify(+math.pi / 2.0, straight_tolerance_rad=TOL) is Maneuver.LEFT
    assert classify(-math.pi / 2.0, straight_tolerance_rad=TOL) is Maneuver.RIGHT


def test_classify_distinguishes_straight_from_uturn() -> None:
    """Косое произведение у обоих около нуля, различает скалярное."""
    small = math.radians(5.0)

    assert classify(small, straight_tolerance_rad=TOL) is Maneuver.STRAIGHT
    assert classify(math.pi - small, straight_tolerance_rad=TOL) is Maneuver.UTURN


def test_four_way_junction_from_south() -> None:
    """Приехал с юга, то есть еду на север."""
    turns = {
        10: turn_angle(NORTH, NORTH),  # луч на север
        20: turn_angle(NORTH, EAST),   # на восток
        30: turn_angle(NORTH, SOUTH),  # назад
        40: turn_angle(NORTH, WEST),   # на запад
    }

    by_target = {
        c.target: c.maneuver
        for c in classify_candidates(turns, straight_tolerance_rad=TOL)
    }

    assert by_target == {
        10: Maneuver.STRAIGHT,
        20: Maneuver.RIGHT,
        30: Maneuver.UTURN,
        40: Maneuver.LEFT,
    }


def test_same_junction_from_east_relabels_the_arms() -> None:
    """Те же лучи, прибытие с востока: метки другие."""
    turns = {
        10: turn_angle(WEST, NORTH),
        20: turn_angle(WEST, EAST),
        30: turn_angle(WEST, SOUTH),
        40: turn_angle(WEST, WEST),
    }

    by_target = {
        c.target: c.maneuver
        for c in classify_candidates(turns, straight_tolerance_rad=TOL)
    }

    assert by_target == {
        10: Maneuver.RIGHT,
        20: Maneuver.UTURN,
        30: Maneuver.LEFT,
        40: Maneuver.STRAIGHT,
    }


def test_skewed_junction_splits_by_order_not_threshold() -> None:
    """Два выхода под -20 и +20: порог назвал бы оба straight."""
    turns = {
        10: math.radians(-20.0),
        20: math.radians(+20.0),
    }

    # Порог их не различает.
    assert classify(turns[10], straight_tolerance_rad=TOL) is Maneuver.STRAIGHT
    assert classify(turns[20], straight_tolerance_rad=TOL) is Maneuver.STRAIGHT

    by_target = {
        c.target: c.maneuver
        for c in classify_candidates(turns, straight_tolerance_rad=TOL)
    }

    # Разделение по порядку угла даёт разные метки.
    assert by_target == {10: Maneuver.RIGHT, 20: Maneuver.LEFT}


def test_three_in_one_class_get_right_straight_left() -> None:
    turns = {
        10: math.radians(-15.0),
        20: math.radians(2.0),
        30: math.radians(+18.0),
    }

    by_target = {
        c.target: c.maneuver
        for c in classify_candidates(turns, straight_tolerance_rad=TOL)
    }

    assert by_target == {
        10: Maneuver.RIGHT,
        20: Maneuver.STRAIGHT,
        30: Maneuver.LEFT,
    }


def test_candidates_are_returned_in_angle_order() -> None:
    turns = {10: math.radians(90.0), 20: math.radians(-90.0), 30: 0.0}

    candidates = classify_candidates(turns, straight_tolerance_rad=TOL)

    assert [c.target for c in candidates] == [20, 30, 10]
    assert [c.turn_deg for c in candidates] == pytest.approx([-90.0, 0.0, 90.0])


def test_no_candidates_gives_empty_result() -> None:
    assert classify_candidates({}, straight_tolerance_rad=TOL) == ()


def test_maneuver_order_is_total_and_covers_all_classes() -> None:
    assert set(MANEUVER_ORDER) == set(Maneuver)
    assert len(MANEUVER_ORDER) == len(Maneuver)


def test_arrival_direction_is_the_last_segment_not_the_chord() -> None:
    """Курс прибытия — то, чем робот подъехал.

    Изогнутая цепочка: прямая между её концами роботу не параллельна,
    поэтому направление прибытия по ней брать нельзя.
    """
    bent = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))

    assert tangent_at_end(bent) == pytest.approx(NORTH)
    # Хорда этой же цепочки смотрит на северо-восток, то есть ни туда ни сюда.
    assert chord_direction(bent[0], bent[-1]) == pytest.approx(
        (0.5 ** 0.5, 0.5 ** 0.5)
    )


def test_arrival_direction_skips_degenerate_segments() -> None:
    with_repeats = ((0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (1.0, 0.0))

    assert tangent_at_end(with_repeats) == pytest.approx(EAST)


def test_direction_of_a_degenerate_pair_is_none() -> None:
    assert tangent_at_end(((1.0, 1.0), (1.0, 1.0))) is None
    assert chord_direction((1.0, 1.0), (1.0, 1.0)) is None


def test_departure_chord_separates_exits_that_share_their_first_edge() -> None:
    """Ради этого отбытие и считается хордой, а не первым сегментом.

    Три цепочки уходят из вершины одним ребром на восток и расходятся
    дальше. По первому сегменту они неразличимы, по хорде — расходятся
    на левый, прямой и правый.
    """
    here = (0.0, 0.0)
    arrival = EAST

    for target, expected in (
        ((1.0, 1.0), +45.0),
        ((1.0, 0.0), 0.0),
        ((1.0, -1.0), -45.0),
    ):
        departure = chord_direction(here, target)
        assert departure is not None
        turn = math.degrees(turn_angle(arrival, departure))
        assert turn == pytest.approx(expected)
