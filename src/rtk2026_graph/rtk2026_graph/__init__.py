"""Дорожный граф и геометрия коридора (вариант B)."""

from rtk2026_graph.geometry import polyline_signed_lateral_m, violates_hard_corridor
from rtk2026_graph.io_geojson import load_geojson_dict, load_geojson_path
from rtk2026_graph.edge_tracking import (
    ActiveEdgeMatch,
    infer_direction_mode_from_yaw,
    select_active_edge_from_limiters,
)
from rtk2026_graph.lane_mode import normalize_lane_mode, opposite_lane_mode
from rtk2026_graph.global_planner_v2 import (
    GlobalPlannerConfigV2,
    GlobalPlannerV2,
    GlobalPlanStepV2,
)
from rtk2026_graph.local_planner_v2 import LaneGoalRuleV2
from rtk2026_graph.local_planner_points_v3 import LocalGoalPointV3, LocalPlannerPointsV3
from rtk2026_graph.planner_v2_config import (
    load_planner_v2_config_dict,
    load_planner_v2_config_path,
)
from rtk2026_graph.lane_goal_geometry import project_goal_on_lane
from rtk2026_graph.model import CorridorHardSide, Node, OrientedEdge, RoadGraph

__all__ = [
    "CorridorHardSide",
    "Node",
    "OrientedEdge",
    "RoadGraph",
    "ActiveEdgeMatch",
    "project_goal_on_lane",
    "load_geojson_dict",
    "load_geojson_path",
    "select_active_edge_from_limiters",
    "infer_direction_mode_from_yaw",
    "normalize_lane_mode",
    "opposite_lane_mode",
    "GlobalPlannerConfigV2",
    "GlobalPlannerV2",
    "GlobalPlanStepV2",
    "LaneGoalRuleV2",
    "LocalGoalPointV3",
    "LocalPlannerPointsV3",
    "load_planner_v2_config_dict",
    "load_planner_v2_config_path",
    "polyline_signed_lateral_m",
    "violates_hard_corridor",
]
