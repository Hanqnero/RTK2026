from rtk2026_graph.corridor import edge_hard_side, violates_hard_corridor
from rtk2026_pose_graph.model import OrientedEdge


def _edge(**metadata: object) -> OrientedEdge:
    return OrientedEdge(edge_id=1, start_id=1, end_id=2, metadata=dict(metadata))


def test_hard_side_read_from_metadata() -> None:
    assert edge_hard_side(_edge(corridor_hard_side="left")) == "left"
    assert edge_hard_side(_edge(corridor_hard_side="right")) == "right"


def test_hard_side_accepts_legacy_key_and_sloppy_case() -> None:
    assert edge_hard_side(_edge(hard_side="RIGHT")) == "right"
    assert edge_hard_side(_edge(corridor_hard_side=" Left ")) == "left"


def test_hard_side_absent_or_unknown_means_no_constraint() -> None:
    assert edge_hard_side(_edge()) is None
    assert edge_hard_side(_edge(corridor_hard_side="middle")) is None
    assert edge_hard_side(_edge(corridor_hard_side=None)) is None


def test_primary_key_wins_over_legacy() -> None:
    edge = _edge(corridor_hard_side="left", hard_side="right")

    assert edge_hard_side(edge) == "left"


def test_no_side_never_violates() -> None:
    assert violates_hard_corridor(lateral_m=100.0, hard_side=None, tol_m=0.1) is False


def test_left_side_blocks_going_further_left() -> None:
    assert violates_hard_corridor(lateral_m=0.5, hard_side="left", tol_m=0.2) is True
    assert violates_hard_corridor(lateral_m=0.1, hard_side="left", tol_m=0.2) is False
    # Право - открытая сторона (costmap/препятствия), туда нарушения нет.
    assert violates_hard_corridor(lateral_m=-5.0, hard_side="left", tol_m=0.2) is False


def test_right_side_blocks_going_further_right() -> None:
    assert violates_hard_corridor(lateral_m=-0.5, hard_side="right", tol_m=0.2) is True
    assert violates_hard_corridor(lateral_m=-0.1, hard_side="right", tol_m=0.2) is False
    assert violates_hard_corridor(lateral_m=5.0, hard_side="right", tol_m=0.2) is False
