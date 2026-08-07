"""Загрузка дорожного графа из GeoJSON FeatureCollection.

Формат совпадает с тем, что читает ``nav2_route::GeoJsonGraphFileLoader``,
поэтому один и тот же файл ``graph.geojson`` годится и Nav2, и этому модулю:

``Point``
    Вершина. Обязательное свойство ``id``, опциональное ``frame``.

``LineString`` / ``MultiLineString``
    Ребро. Обязательные свойства ``id``, ``startid``, ``endid``.
    Опциональные ``cost``, ``overridable``. Геометрию можно не задавать —
    тогда ребро становится прямой между своими вершинами.

Всё остальное из ``properties`` попадает в ``metadata`` как есть.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph

#: Свойства, которые загрузчик разбирает сам. В ``metadata`` они не дублируются.
_NODE_STRUCTURAL_KEYS = frozenset({"id", "frame"})
_EDGE_STRUCTURAL_KEYS = frozenset({"id", "startid", "endid", "cost", "overridable"})


def load_geojson_path(path: str | Path) -> RoadGraph:
    """Прочитать граф из файла на диске."""
    with Path(path).open(encoding="utf-8") as stream:
        return load_geojson_dict(json.load(stream))


def load_geojson_dict(data: dict[str, Any]) -> RoadGraph:
    """Собрать граф из разобранного GeoJSON FeatureCollection.

    Вершины читаются первым проходом, рёбра вторым: ребру без собственной
    геометрии нужны координаты своих вершин.

    :raises ValueError: структура не FeatureCollection, дублируется ``id``
        вершины, либо у ребра нет ни геометрии, ни существующих вершин.
    """
    if data.get("type") != "FeatureCollection":
        raise ValueError("ожидается type=FeatureCollection")

    features = data.get("features")
    if not isinstance(features, list):
        raise ValueError("features должен быть списком")

    nodes: dict[int, Node] = {}
    edges: dict[int, OrientedEdge] = {}

    for feature in _iter_features(features):
        geometry, properties = feature
        if geometry.get("type") == "Point":
            _parse_node(geometry, properties, nodes)

    for feature in _iter_features(features):
        geometry, properties = feature
        if geometry.get("type") in ("LineString", "MultiLineString"):
            _parse_edge(geometry, properties, nodes, edges)

    return RoadGraph(nodes=nodes, edges=edges)


def _iter_features(
    features: list[Any],
) -> list[tuple[dict[str, Any], dict[str, Any]]]:
    """Отобрать корректные Feature и вернуть пары (geometry, properties)."""
    out: list[tuple[dict[str, Any], dict[str, Any]]] = []
    for feature in features:
        if not isinstance(feature, dict) or feature.get("type") != "Feature":
            continue
        geometry = feature.get("geometry") or {}
        properties = feature.get("properties") or {}
        if isinstance(geometry, dict) and isinstance(properties, dict):
            out.append((geometry, properties))
    return out


def _parse_node(
    geometry: dict[str, Any],
    properties: dict[str, Any],
    nodes: dict[int, Node],
) -> None:
    if "id" not in properties:
        return

    coordinates = geometry.get("coordinates")
    if not isinstance(coordinates, (list, tuple)) or len(coordinates) < 2:
        return

    node_id = int(properties["id"])
    if node_id in nodes:
        raise ValueError(f"дубликат id узла: {node_id}")

    nodes[node_id] = Node(
        node_id=node_id,
        x=float(coordinates[0]),
        y=float(coordinates[1]),
        frame=str(properties.get("frame", "map")),
        metadata=_extra_properties(properties, _NODE_STRUCTURAL_KEYS),
    )


def _parse_edge(
    geometry: dict[str, Any],
    properties: dict[str, Any],
    nodes: dict[int, Node],
    edges: dict[int, OrientedEdge],
) -> None:
    if not {"id", "startid", "endid"} <= properties.keys():
        return

    edge_id = int(properties["id"])
    if edge_id in edges:
        return

    start_id = int(properties["startid"])
    end_id = int(properties["endid"])
    polyline = _polyline_for_edge(geometry, edge_id, start_id, end_id, nodes)

    edges[edge_id] = OrientedEdge(
        edge_id=edge_id,
        start_id=start_id,
        end_id=end_id,
        polyline_xy=polyline,
        cost=float(properties.get("cost", 0.0)),
        overridable=bool(properties.get("overridable", True)),
        metadata=_extra_properties(properties, _EDGE_STRUCTURAL_KEYS),
    )


def _polyline_for_edge(
    geometry: dict[str, Any],
    edge_id: int,
    start_id: int,
    end_id: int,
    nodes: dict[int, Node],
) -> tuple[tuple[float, float], ...]:
    """Полилиния ребра: из геометрии, иначе прямая между вершинами."""
    polyline = _coordinates_to_polyline(geometry)

    if not polyline:
        if start_id not in nodes or end_id not in nodes:
            raise ValueError(
                f"ребро {edge_id}: нет координат геометрии "
                f"и не найдены узлы {start_id}->{end_id}"
            )
        polyline = [nodes[start_id].xy, nodes[end_id].xy]

    if len(polyline) < 2:
        raise ValueError(f"ребро {edge_id}: нужна полилиния из минимум двух точек")

    return tuple(polyline)


def _coordinates_to_polyline(geometry: dict[str, Any]) -> list[tuple[float, float]]:
    """Точки геометрии ребра. У MultiLineString берётся первая линия."""
    coordinates = geometry.get("coordinates")
    if not isinstance(coordinates, list) or not coordinates:
        return []

    if geometry.get("type") == "MultiLineString":
        first_line = coordinates[0]
        coordinates = first_line if isinstance(first_line, list) else []

    out: list[tuple[float, float]] = []
    for point in coordinates:
        if isinstance(point, (list, tuple)) and len(point) >= 2:
            out.append((float(point[0]), float(point[1])))
    return out


def _extra_properties(
    properties: dict[str, Any],
    structural_keys: frozenset[str],
) -> dict[str, Any]:
    """Все свойства, кроме разобранных загрузчиком."""
    return {
        key: value for key, value in properties.items() if key not in structural_keys
    }
