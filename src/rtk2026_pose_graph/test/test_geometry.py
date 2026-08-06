from rtk2026_pose_graph.geometry import polyline_signed_lateral_m, violates_hard_corridor

_HORIZONTAL = ((0.0, 0.0), (1.0, 0.0))


def test_polyline_signed_lateral_positive_on_left() -> None:
    # Правило правой руки: точка выше сегмента (0,0)->(1,0) лежит слева.
    assert polyline_signed_lateral_m(0.5, 1.0, _HORIZONTAL) == 1.0


def test_polyline_signed_lateral_negative_on_right() -> None:
    assert polyline_signed_lateral_m(0.5, -1.0, _HORIZONTAL) == -1.0


def test_polyline_signed_lateral_picks_nearest_segment() -> None:
    polyline = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))

    # (1.5, 0.5) однозначно ближе ко второму сегменту (1,0)->(1,1)
    # (dist^2 0.25 против 0.5 до первого); справа от него - x > 1.
    assert polyline_signed_lateral_m(1.5, 0.5, polyline) == -0.5


def test_polyline_signed_lateral_empty_polyline_is_zero() -> None:
    assert polyline_signed_lateral_m(1.0, 1.0, ()) == 0.0
    assert polyline_signed_lateral_m(1.0, 1.0, ((0.0, 0.0),)) == 0.0


def test_violates_hard_corridor_none_side_never_violates() -> None:
    assert violates_hard_corridor(lateral_m=100.0, hard_side=None, tol_m=0.1) is False


def test_violates_hard_corridor_left_side_blocks_going_further_left() -> None:
    assert violates_hard_corridor(lateral_m=0.5, hard_side="left", tol_m=0.2) is True
    assert violates_hard_corridor(lateral_m=0.1, hard_side="left", tol_m=0.2) is False
    # Право - открытая сторона (costmap/препятствия), туда нарушения нет.
    assert violates_hard_corridor(lateral_m=-5.0, hard_side="left", tol_m=0.2) is False


def test_violates_hard_corridor_right_side_blocks_going_further_right() -> None:
    assert violates_hard_corridor(lateral_m=-0.5, hard_side="right", tol_m=0.2) is True
    assert violates_hard_corridor(lateral_m=-0.1, hard_side="right", tol_m=0.2) is False
    assert violates_hard_corridor(lateral_m=5.0, hard_side="right", tol_m=0.2) is False
