#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path

import yaml

from rtk2026_graph import load_geojson_path
from rtk2026_graph.lane_mode import normalize_lane_mode


def _edge_polyline_or_reversed(graph, start_id: int, end_id: int):
    edge = graph.edge_toward_neighbor(int(start_id), int(end_id))
    if edge is not None and edge.polyline_xy:
        return tuple(edge.polyline_xy)
    back = graph.edge_toward_neighbor(int(end_id), int(start_id))
    if back is not None and back.polyline_xy:
        return tuple(reversed(back.polyline_xy))
    return None


def _chain_polyline(graph, limiter_edges):
    merged = []
    for idx, (a, b) in enumerate(limiter_edges):
        poly = _edge_polyline_or_reversed(graph, int(a), int(b))
        if not poly or len(poly) < 2:
            return None
        if idx == 0:
            merged.extend(poly)
        else:
            px, py = merged[-1]
            qx, qy = poly[0]
            if abs(px - qx) < 1e-6 and abs(py - qy) < 1e-6:
                merged.extend(poly[1:])
            else:
                merged.extend(poly)
    return tuple(merged) if len(merged) >= 2 else None


def _sample_at_fraction(polyline, fraction: float):
    if len(polyline) < 2:
        return None
    seg_lens = []
    cum = [0.0]
    for i in range(len(polyline) - 1):
        x0, y0 = polyline[i]
        x1, y1 = polyline[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        seg_lens.append(seg)
        cum.append(cum[-1] + seg)
    total = cum[-1]
    if total <= 1e-9:
        return None
    s = max(0.0, min(1.0, float(fraction))) * total
    seg_idx = len(seg_lens) - 1
    for i in range(len(seg_lens)):
        if s <= cum[i + 1]:
            seg_idx = i
            break
    seg_len = seg_lens[seg_idx]
    if seg_len <= 1e-9:
        return None
    t = (s - cum[seg_idx]) / seg_len
    t = max(0.0, min(1.0, t))
    x0, y0 = polyline[seg_idx]
    x1, y1 = polyline[seg_idx + 1]
    gx = x0 + (x1 - x0) * t
    gy = y0 + (y1 - y0) * t
    tx = x1 - x0
    ty = y1 - y0
    norm = math.hypot(tx, ty)
    if norm <= 1e-9:
        tx, ty = 1.0, 0.0
    else:
        tx, ty = tx / norm, ty / norm
    return gx, gy, tx, ty


def _offset_point(sample, sign: int, offset_m: float):
    gx, gy, tx, ty = sample
    right_x, right_y = ty, -tx
    s = -1.0 if int(sign) == -1 else 1.0
    return gx + right_x * offset_m * s, gy + right_y * offset_m * s


def _lane_sign(rule: dict, lane_mode: str) -> int:
    signs = rule.get("lane_goal_sign_by_lane", {}) or {}
    val = signs.get(lane_mode)
    if val in (-1, 1):
        return int(val)
    # Совместимо с текущим runtime: lane2 по умолчанию справа.
    return 1


def main() -> None:
    repo = Path(__file__).resolve().parents[1]
    cfg_path = repo / "src/rtk2026_route_nav/config/lane_planner_v2.yaml"
    graph_path = repo / "src/rtk2026_route_nav/config/graph.geojson"
    with cfg_path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    graph = load_geojson_path(graph_path)

    rules = data.get("local_goal_rules", [])
    if not isinstance(rules, list):
        raise ValueError("local_goal_rules must be a list")

    for rule in rules:
        if not isinstance(rule, dict):
            continue
        edges = rule.get("limiter_edges", [])
        if not isinstance(edges, list) or not edges:
            continue
        edge_pairs = []
        for e in edges:
            if isinstance(e, (list, tuple)) and len(e) >= 2:
                edge_pairs.append((int(e[0]), int(e[1])))
        if not edge_pairs:
            continue
        poly = _chain_polyline(graph, edge_pairs)
        if poly is None:
            continue

        if len(edge_pairs) >= 2:
            fractions = (0.45, 0.9)
        else:
            fractions = (0.92,)

        lane_points = {}
        for lane_mode in ("lane1", "lane2"):
            sign = _lane_sign(rule, normalize_lane_mode(lane_mode))
            points = []
            for fr in fractions:
                sample = _sample_at_fraction(poly, fr)
                if sample is None:
                    continue
                px, py = _offset_point(sample, sign=sign, offset_m=0.20)
                points.append([round(px, 3), round(py, 3)])
            if points:
                lane_points[lane_mode] = points

        if lane_points:
            rule["nav2_waypoints_by_lane"] = lane_points
            # Для нового формата не дублируем одиночную точку в старом ключе.
            rule.pop("nav2_goal_by_lane", None)

    with cfg_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

    print(f"Updated {cfg_path}")


if __name__ == "__main__":
    main()

