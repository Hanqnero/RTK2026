"""Дорожный граф на карте: вершины, ориентированные рёбра, геометрия, загрузка.

Прикладные свойства вершин и рёбер хранятся в ``metadata`` и трактуются
тем кодом, которому нужны.

Формат GeoJSON совпадает с ``nav2_route::GeoJsonGraphFileLoader``, поэтому
один и тот же файл графа читают и Nav2, и этот модуль.
"""

from rtk2026_pose_graph.geometry import (
    PolylineProjection,
    interpolate_along_polyline,
    point_to_polyline_distance_m,
    polyline_length_m,
    polyline_signed_lateral_m,
    project_point_on_polyline,
)
from rtk2026_pose_graph.io_geojson import load_geojson_dict, load_geojson_path
from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph

__all__ = [
    "Node",
    "OrientedEdge",
    "RoadGraph",
    "PolylineProjection",
    "project_point_on_polyline",
    "polyline_signed_lateral_m",
    "point_to_polyline_distance_m",
    "polyline_length_m",
    "interpolate_along_polyline",
    "load_geojson_dict",
    "load_geojson_path",
]
