"""Загрузка FeatureCollection (формат как у RTK2026/routes/graph-101) в RoadGraph."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from rtk2026_pose_graph.model import CorridorHardSide, Node, OrientedEdge, RoadGraph


def load_geojson_path(path: str | Path) -> RoadGraph:
    """Читает JSON с диска и строит RoadGraph."""
    p = Path(path)
    with p.open(encoding="utf-8") as f:
        data = json.load(f)
    return load_geojson_dict(data)


def load_geojson_dict(data: dict[str, Any]) -> RoadGraph:
    """Разбирает GeoJSON FeatureCollection: точки (узлы) и рёбра с startid/endid."""
    if data.get("type") != "FeatureCollection":
        raise ValueError("ожидается type=FeatureCollection")

    features = data.get("features")
    if not isinstance(features, list):
        raise ValueError("features должен быть списком")

    nodes: dict[int, Node] = {}
    edges: dict[int, OrientedEdge] = {}

    for feat in features:
        if not isinstance(feat, dict) or feat.get("type") != "Feature":
            continue
        geom = feat.get("geometry") or {}
        props = feat.get("properties") or {}
        if geom.get("type") == "Point":
            _parse_point_feature(geom, props, nodes)

    for feat in features:
        if not isinstance(feat, dict) or feat.get("type") != "Feature":
            continue
        geom = feat.get("geometry") or {}
        props = feat.get("properties") or {}
        gtype = geom.get("type")
        if gtype in ("LineString", "MultiLineString"):
            _parse_edge_feature(geom, props, gtype, nodes, edges)

    return RoadGraph(nodes=nodes, edges=edges)


def _geometry_has_coordinates(geom: dict[str, Any]) -> bool:
    c = geom.get("coordinates")
    if c is None:
        return False
    if isinstance(c, list) and len(c) > 0:
        return True
    return False


def _parse_point_feature(
    geom: dict[str, Any],
    props: dict[str, Any],
    nodes: dict[int, Node],
) -> None:
    if "id" not in props:
        return
    nid = int(props["id"])
    coords = geom.get("coordinates")
    if not isinstance(coords, (list, tuple)) or len(coords) < 2:
        return
    x, y = float(coords[0]), float(coords[1])
    frame = str(props.get("frame", "map"))
    if nid in nodes:
        raise ValueError(f"дубликат id узла: {nid}")
    nodes[nid] = Node(node_id=nid, x=x, y=y, frame=frame)


def _parse_edge_feature(
    geom: dict[str, Any],
    props: dict[str, Any],
    gtype: str,
    nodes: dict[int, Node],
    edges: dict[int, OrientedEdge],
) -> None:
    if "startid" not in props or "endid" not in props or "id" not in props:
        return
    edge_id = int(props["id"])
    start_id = int(props["startid"])
    end_id = int(props["endid"])
    if edge_id in edges:
        return

    cost = float(props.get("cost", 0.0))
    overridable = bool(props.get("overridable", True))
    hard = _parse_corridor_hard_side(props)

    poly: list[tuple[float, float]] = []
    if _geometry_has_coordinates(geom):
        poly = _coords_to_polyline_xy(geom, gtype)
    else:
        if start_id not in nodes or end_id not in nodes:
            raise ValueError(
                f"ребро {edge_id}: нет координат геометрии и не найдены узлы {start_id}→{end_id}"
            )
        a, b = nodes[start_id], nodes[end_id]
        poly = [(a.x, a.y), (b.x, b.y)]

    if len(poly) < 2:
        raise ValueError(f"ребро {edge_id}: нужна полилиния из минимум двух точек")

    edges[edge_id] = OrientedEdge(
        edge_id=edge_id,
        start_id=start_id,
        end_id=end_id,
        polyline_xy=tuple(poly),
        cost=cost,
        overridable=overridable,
        corridor_hard_side=hard,
    )


def _coords_to_polyline_xy(geom: dict[str, Any], gtype: str) -> list[tuple[float, float]]:
    c = geom["coordinates"]
    if gtype == "LineString":
        if not isinstance(c, list):
            return []
        out: list[tuple[float, float]] = []
        for pt in c:
            if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                out.append((float(pt[0]), float(pt[1])))
        return out
    if gtype == "MultiLineString":
        if not isinstance(c, list) or not c:
            return []
        line0 = c[0]
        if not isinstance(line0, list):
            return []
        out = []
        for pt in line0:
            if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                out.append((float(pt[0]), float(pt[1])))
        return out
    return []


def _parse_corridor_hard_side(props: dict[str, Any]) -> CorridorHardSide | None:
    raw = props.get("corridor_hard_side", props.get("hard_side"))
    if raw is None:
        return None
    s = str(raw).strip().lower()
    if s == "left":
        return "left"
    if s == "right":
        return "right"
    return None
