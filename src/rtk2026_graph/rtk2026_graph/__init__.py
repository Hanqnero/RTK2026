"""Алгоритм движения по дорожному графу: выбор ребра, планировщики, полосы.

Сама структура графа — вершины, ориентированные рёбра, геометрия полилинии,
загрузка из GeoJSON — вынесена в ``rtk2026_pose_graph``. Тот модуль общий:
не знает ни про позу робота, ни про коридоры, ни про полосы, и переживёт
переписывание алгоритма в этом пакете.

Прикладной смысл рёбер приходит из ``metadata`` графа. Правила его
трактовки лежат здесь: например, сторона непересекаемой границы —
в :mod:`rtk2026_graph.corridor`.
"""

from rtk2026_pose_graph import (
    Node,
    OrientedEdge,
    RoadGraph,
    load_geojson_dict,
    load_geojson_path,
    point_to_polyline_distance_m,
    polyline_signed_lateral_m,
)
from rtk2026_graph.corridor import (
    CorridorHardSide,
    edge_hard_side,
    violates_hard_corridor,
)
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

__all__ = [
    # Реэкспорт общего графа: чтобы существующие потребители не переписывали
    # импорты из-за выноса модели.
    "Node",
    "OrientedEdge",
    "RoadGraph",
    "load_geojson_dict",
    "load_geojson_path",
    "point_to_polyline_distance_m",
    "polyline_signed_lateral_m",
    # Правила этого пакета.
    "CorridorHardSide",
    "edge_hard_side",
    "violates_hard_corridor",
    "ActiveEdgeMatch",
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
    "project_goal_on_lane",
]
