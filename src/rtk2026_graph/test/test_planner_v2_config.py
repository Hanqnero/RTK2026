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
            "sign_direction_topology": {
                "lane_vertices": {
                    "lane1": {"4": {"straight": 2, "right": 17, "left": None}},
                },
                "intersection_vertices": {
                    "17": {
                        "entry": {"straight": 19, "right": 18, "left": 20},
                        "exit": {"right": 2, "left": 4},
                    }
                },
            },
            "sign_command_mapping": {
                "straight_only": {"preferred": ["straight"], "forbidden": []},
                "no_right_turn": {"preferred": [], "forbidden": ["right"]},
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
                }
            ],
        }
    )
    assert global_cfg.initial_lane_mode == "lane1"
    assert global_cfg.lane_targets["lane2"][8] == (10,)
    assert global_cfg.intersection_exit_lane_targets["lane1"][17] == (2,)
    assert global_cfg.intersection_exit_lane_targets["lane2"][17] == (4,)
    assert global_cfg.lane_direction_targets["lane1"][4]["straight"] == 2
    assert global_cfg.intersection_direction_targets[17]["entry"]["left"] == 20
    assert global_cfg.sign_command_mapping["straight_only"]["preferred"] == ("straight",)
    assert local_rules[0].limiter_edges == ((2, 1), (1, 16))
