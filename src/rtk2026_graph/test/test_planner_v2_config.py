import pytest

from rtk2026_graph.lane_mode import normalize_lane_mode
from rtk2026_graph.planner_v2_config import load_planner_v2_config_dict


def test_lane_mode_accepts_only_lane1_lane2() -> None:
    assert normalize_lane_mode("lane1") == "lane1"
    assert normalize_lane_mode("lane2") == "lane2"
    with pytest.raises(ValueError):
        normalize_lane_mode("forward")


def test_load_planner_v2_config_dict() -> None:
    global_cfg, local_rules = load_planner_v2_config_dict(
        {
            "initial_lane_mode": "lane1",
            "intersection_vertex_ids": [17, 18, 19, 20],
            "lane_switch_edges": [[18, 8]],
            "intersection_exit_lane_targets": {
                "lane1": {"17": [2]},
                "lane2": {"17": [4]},
            },
            "lane_targets": {
                "lane1": {"8": [18, 6]},
                "lane2": {"8": [10]},
            },
            "local_limiter_edges": [
                {
                    "current_vertex": 2,
                    "target_vertex": 16,
                    "limiter_edges": [[2, 1], [1, 16]],
                }
            ],
            "local_goal_rules": [
                {
                    "current_vertex": 2,
                    "target_vertex": 16,
                    "lane_goal_sign_by_lane": {"lane1": -1, "lane2": 1},
                    "nav2_goal_by_lane": {"lane2": [1.2, -0.3, 0.5]},
                    "nav2_waypoints_by_lane": {"lane1": [[0.1, 0.2], [0.3, 0.4, 1.0]]},
                }
            ],
        }
    )
    assert global_cfg.initial_lane_mode == "lane1"
    assert global_cfg.lane_targets["lane2"][8] == (10,)
    assert global_cfg.intersection_exit_lane_targets["lane1"][17] == (2,)
    assert global_cfg.intersection_exit_lane_targets["lane2"][17] == (4,)
    assert local_rules[0].lane_goal_sign_by_lane["lane1"] == -1
    assert local_rules[0].limiter_edges == ((2, 1), (1, 16))
    assert local_rules[0].nav2_goal_by_lane["lane2"] == (1.2, -0.3, 0.5)
    assert local_rules[0].nav2_waypoints_by_lane["lane1"] == ((0.1, 0.2, None), (0.3, 0.4, 1.0))
