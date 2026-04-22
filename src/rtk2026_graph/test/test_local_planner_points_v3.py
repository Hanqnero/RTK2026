import math

from rtk2026_graph.local_planner_points_v3 import LocalPlannerPointsV3
from rtk2026_graph.local_planner_v2 import LaneGoalRuleV2
from rtk2026_graph.model import Node, OrientedEdge, RoadGraph


def _graph() -> RoadGraph:
    nodes = {
        1: Node(node_id=1, x=0.0, y=0.0),
        2: Node(node_id=2, x=1.0, y=0.0),
        4: Node(node_id=4, x=1.0, y=1.0),
    }
    edges = {
        1: OrientedEdge(edge_id=1, start_id=1, end_id=2, polyline_xy=((0.0, 0.0), (1.0, 0.0))),
        2: OrientedEdge(edge_id=2, start_id=2, end_id=4, polyline_xy=((1.0, 0.0), (1.0, 1.0))),
    }
    return RoadGraph(nodes=nodes, edges=edges)

def _straight_two_edge_graph() -> RoadGraph:
    nodes = {
        4: Node(node_id=4, x=-0.65, y=-0.157),
        3: Node(node_id=3, x=-1.45, y=-0.157),
        2: Node(node_id=2, x=-2.25, y=-0.157),
    }
    return RoadGraph(nodes=nodes, edges={})


def test_v3_single_edge_has_midpoint_and_endpoint() -> None:
    planner = LocalPlannerPointsV3(
        _graph(),
        (
            LaneGoalRuleV2(
                current_vertex=1,
                target_vertex=2,
                limiter_edges=((1, 2),),
            ),
        ),
    )
    seq = planner.build_goal_sequence(current_vertex=1, target_vertex=2, lane_mode="lane1")
    assert len(seq) == 2
    assert seq[0].waypoint_index == 0
    assert seq[1].waypoint_index == 1
    assert seq[1].waypoint_count == 2


def test_v3_two_edge_turn_has_four_points() -> None:
    planner = LocalPlannerPointsV3(
        _graph(),
        (
            LaneGoalRuleV2(
                current_vertex=1,
                target_vertex=4,
                limiter_edges=((1, 2), (2, 4)),
            ),
        ),
    )
    seq = planner.build_goal_sequence(current_vertex=1, target_vertex=4, lane_mode="lane2")
    assert len(seq) == 4
    assert all(math.isfinite(p.yaw) for p in seq)


def test_v3_two_edge_collinear_has_midpoints_and_endpoint() -> None:
    planner = LocalPlannerPointsV3(
        _straight_two_edge_graph(),
        (
            LaneGoalRuleV2(
                current_vertex=4,
                target_vertex=2,
                limiter_edges=((4, 3), (3, 2)),
            ),
        ),
    )
    seq = planner.build_goal_sequence(current_vertex=4, target_vertex=2, lane_mode="lane1")
    assert len(seq) == 3
    assert seq[0].x > seq[1].x > seq[2].x
    assert seq[0].waypoint_count == 3
    assert seq[-1].waypoint_index == 2


def test_v3_offset_is_always_on_right_side_of_oriented_edge() -> None:
    planner = LocalPlannerPointsV3(
        _graph(),
        (
            LaneGoalRuleV2(
                current_vertex=1,
                target_vertex=2,
                limiter_edges=((1, 2),),
            ),
        ),
    )
    lane1 = planner.build_goal_sequence(current_vertex=1, target_vertex=2, lane_mode="lane1")[0]
    lane2 = planner.build_goal_sequence(current_vertex=1, target_vertex=2, lane_mode="lane2")[0]
    assert lane1.lane_goal_sign == 1
    assert lane2.lane_goal_sign == 1
    assert lane1.y < 0.0
    assert lane2.y < 0.0


def test_v3_lane_mode_does_not_flip_offset_side() -> None:
    planner = LocalPlannerPointsV3(
        _graph(),
        (
            LaneGoalRuleV2(
                current_vertex=1,
                target_vertex=2,
                limiter_edges=((1, 2),),
            ),
        ),
    )
    lane1 = planner.build_goal_sequence(current_vertex=1, target_vertex=2, lane_mode="lane1")[0]
    lane2 = planner.build_goal_sequence(current_vertex=1, target_vertex=2, lane_mode="lane2")[0]
    assert lane1.lane_goal_sign == 1
    assert lane2.lane_goal_sign == 1
    assert lane1.y < 0.0
    assert lane2.y < 0.0
