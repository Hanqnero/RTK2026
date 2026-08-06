"""Дорожный граф: вершины, ориентированные рёбра, геометрия коридора, загрузка из GeoJSON.

Чистая структура данных без ROS и без решений о движении: не знает про позу
робота, топики или планировщики. Это осознанная граница — код, принимающий
решения по графу (выбор активного ребра, построение маршрута), живёт
в ``rtk2026_graph`` и других пакетах, которые этот граф используют.
"""

from rtk2026_pose_graph.geometry import polyline_signed_lateral_m, violates_hard_corridor
from rtk2026_pose_graph.io_geojson import load_geojson_dict, load_geojson_path
from rtk2026_pose_graph.model import CorridorHardSide, Node, OrientedEdge, RoadGraph

__all__ = [
    "CorridorHardSide",
    "Node",
    "OrientedEdge",
    "RoadGraph",
    "load_geojson_dict",
    "load_geojson_path",
    "polyline_signed_lateral_m",
    "violates_hard_corridor",
]
