import math

from rtk2026_graph.edge_tracking import (
    infer_direction_mode_from_yaw,
    select_active_edge_from_limiters,
)
from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph


def _build_graph() -> RoadGraph:
    nodes = {
        1: Node(node_id=1, x=0.0, y=0.0),
        2: Node(node_id=2, x=1.0, y=0.0),
        3: Node(node_id=3, x=1.0, y=1.0),
    }
    edges = {
        100: OrientedEdge(
            edge_id=100,
            start_id=1,
            end_id=2,
            polyline_xy=((0.0, 0.0), (1.0, 0.0)),
        ),
        101: OrientedEdge(
            edge_id=101,
            start_id=2,
            end_id=3,
            polyline_xy=((1.0, 0.0), (1.0, 1.0)),
        ),
    }
    return RoadGraph(nodes=nodes, edges=edges)


def test_select_active_edge_prefers_nearest_limiter_edge() -> None:
    graph = _build_graph()
    limiter_edges = ((1, 2), (2, 3))

    # Ближе к горизонтальному ребру 1->2.
    match = select_active_edge_from_limiters(
        graph=graph,
        limiter_edges=limiter_edges,
        pose_x=0.3,
        pose_y=0.05,
        yaw_rad=0.0,
    )

    assert match is not None
    assert (match.start_id, match.end_id) == (1, 2)


def test_select_active_edge_uses_heading_as_tie_breaker() -> None:
    graph = _build_graph()
    limiter_edges = ((1, 2), (2, 3))

    # Рядом с точкой пересечения, дистанции почти одинаковые.
    # Yaw вдоль вертикали, значит должен выбрать 2->3.
    match = select_active_edge_from_limiters(
        graph=graph,
        limiter_edges=limiter_edges,
        pose_x=1.0,
        pose_y=0.0,
        yaw_rad=math.pi / 2.0,
    )

    assert match is not None
    assert (match.start_id, match.end_id) == (2, 3)


def test_infer_direction_mode_from_yaw_forward_reverse() -> None:
    graph = _build_graph()
    edge = graph.edge_toward_neighbor(1, 2)
    assert edge is not None

    assert infer_direction_mode_from_yaw(edge, yaw_rad=0.0) == "forward"
    assert infer_direction_mode_from_yaw(edge, yaw_rad=math.pi) == "reverse"
