"""Загрузка конфигурации v2-планеров из YAML."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from rtk2026_graph.global_planner_v2 import GlobalPlannerConfigV2
from rtk2026_graph.local_planner_v2 import LaneGoalRuleV2
from rtk2026_graph.lane_mode import normalize_lane_mode


def load_planner_v2_config_path(path: str | Path) -> tuple[GlobalPlannerConfigV2, tuple[LaneGoalRuleV2, ...]]:
    p = Path(path)
    with p.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("planner v2 config: YAML root must be map")
    # Optional: вынесенные limiter_edges в отдельный YAML-файл.
    external_edges_file = data.get("local_limiter_edges_file")
    if isinstance(external_edges_file, str) and external_edges_file.strip():
        ext_path = Path(external_edges_file.strip())
        if not ext_path.is_absolute():
            ext_path = (p.parent / ext_path).resolve()
        with ext_path.open(encoding="utf-8") as ef:
            ext_data = yaml.safe_load(ef) or {}
        if not isinstance(ext_data, dict):
            raise ValueError("planner v2 limiter edges: YAML root must be map")
        # Вставляем как inline-структуру, чтобы общий парсер работал одинаково.
        data["local_limiter_edges"] = ext_data.get("local_limiter_edges", [])
    return load_planner_v2_config_dict(data)


def load_planner_v2_config_dict(
    data: dict[str, Any],
) -> tuple[GlobalPlannerConfigV2, tuple[LaneGoalRuleV2, ...]]:
    global_cfg = GlobalPlannerConfigV2(
        lane_targets=_parse_lane_targets(data.get("lane_targets", {})),
        intersection_vertex_ids=frozenset(int(v) for v in data.get("intersection_vertex_ids", [])),
        lane_switch_edges=_parse_lane_switch_edges(data.get("lane_switch_edges", [])),
        intersection_exit_lane_targets=_parse_intersection_exit_lane_targets(
            data.get("intersection_exit_lane_targets", {})
        ),
        initial_lane_mode=normalize_lane_mode(str(data.get("initial_lane_mode", "lane1"))),
    )
    limiter_edges_index = _parse_local_limiter_edges(data.get("local_limiter_edges", []))
    local_rules = _parse_local_goal_rules(data.get("local_goal_rules", []), limiter_edges_index=limiter_edges_index)
    return global_cfg, local_rules


def _parse_lane_targets(raw: Any) -> dict[str, dict[int, tuple[int, ...]]]:
    out: dict[str, dict[int, tuple[int, ...]]] = {}
    if not isinstance(raw, dict):
        return out
    for lane_mode, by_vertex in raw.items():
        if not isinstance(by_vertex, dict):
            continue
        mode = normalize_lane_mode(str(lane_mode))
        out[mode] = {}
        for vertex, targets in by_vertex.items():
            if not isinstance(targets, list):
                continue
            out[mode][int(vertex)] = tuple(int(v) for v in targets)
    return out


def _parse_lane_switch_edges(raw: Any) -> frozenset[tuple[int, int]]:
    if not isinstance(raw, list):
        return frozenset()
    out: set[tuple[int, int]] = set()
    for item in raw:
        if isinstance(item, (list, tuple)) and len(item) >= 2:
            out.add((int(item[0]), int(item[1])))
    return frozenset(out)


def _parse_intersection_exit_lane_targets(raw: Any) -> dict[str, dict[int, tuple[int, ...]]]:
    out: dict[str, dict[int, tuple[int, ...]]] = {}
    if not isinstance(raw, dict):
        return out
    for lane_mode, by_vertex in raw.items():
        if not isinstance(by_vertex, dict):
            continue
        mode = normalize_lane_mode(str(lane_mode))
        out[mode] = {}
        for vertex, targets in by_vertex.items():
            if not isinstance(targets, list):
                continue
            out[mode][int(vertex)] = tuple(int(v) for v in targets)
    return out


def _parse_local_limiter_edges(raw: Any) -> dict[tuple[int, int], tuple[tuple[int, int], ...]]:
    out: dict[tuple[int, int], tuple[tuple[int, int], ...]] = {}
    if not isinstance(raw, list):
        return out
    for item in raw:
        if not isinstance(item, dict):
            continue
        current = int(item.get("current_vertex"))
        target = int(item.get("target_vertex"))
        edges = item.get("limiter_edges", [])
        parsed: list[tuple[int, int]] = []
        if isinstance(edges, list):
            for e in edges:
                if isinstance(e, (list, tuple)) and len(e) >= 2:
                    parsed.append((int(e[0]), int(e[1])))
        if parsed:
            out[(current, target)] = tuple(parsed)
    return out


def _parse_local_goal_rules(
    raw: Any,
    *,
    limiter_edges_index: dict[tuple[int, int], tuple[tuple[int, int], ...]],
) -> tuple[LaneGoalRuleV2, ...]:
    if not isinstance(raw, list):
        return ()
    out: list[LaneGoalRuleV2] = []
    for item in raw:
        if not isinstance(item, dict):
            continue
        lane_signs_raw = item.get("lane_goal_sign_by_lane", {})
        lane_signs: dict[str, int] = {}
        if isinstance(lane_signs_raw, dict):
            for lane_mode, sign in lane_signs_raw.items():
                if int(sign) in (-1, 1):
                    lane_signs[normalize_lane_mode(str(lane_mode))] = int(sign)
        prev_signs_raw = item.get("lane_goal_sign_by_previous", {})
        prev_signs: dict[int, int] = {}
        if isinstance(prev_signs_raw, dict):
            for prev, sign in prev_signs_raw.items():
                if int(sign) in (-1, 1):
                    prev_signs[int(prev)] = int(sign)
        current_vertex = int(item.get("current_vertex"))
        target_vertex = int(item.get("target_vertex"))
        indexed_edges = limiter_edges_index.get((current_vertex, target_vertex), ())
        limiter_edges: list[tuple[int, int]] = list(indexed_edges)
        # Backward-compatible fallback: брать limiter_edges из local_goal_rules,
        # только если для пары нет записи в вынесенном индексе.
        if not limiter_edges:
            edges = item.get("limiter_edges", [])
            if isinstance(edges, list):
                for e in edges:
                    if isinstance(e, (list, tuple)) and len(e) >= 2:
                        limiter_edges.append((int(e[0]), int(e[1])))
        nav2_goal_raw = item.get("nav2_goal_by_lane", {})
        nav2_goal_by_lane: dict[str, tuple[float, float, float | None]] = {}
        if isinstance(nav2_goal_raw, dict):
            for lane_mode, goal in nav2_goal_raw.items():
                if not isinstance(goal, (list, tuple)) or len(goal) < 2:
                    continue
                mode = normalize_lane_mode(str(lane_mode))
                gx = float(goal[0])
                gy = float(goal[1])
                gyaw = float(goal[2]) if len(goal) >= 3 and goal[2] is not None else None
                nav2_goal_by_lane[mode] = (gx, gy, gyaw)
        nav2_waypoints_raw = item.get("nav2_waypoints_by_lane", {})
        nav2_waypoints_by_lane: dict[str, tuple[tuple[float, float, float | None], ...]] = {}
        if isinstance(nav2_waypoints_raw, dict):
            for lane_mode, points in nav2_waypoints_raw.items():
                if not isinstance(points, list):
                    continue
                mode = normalize_lane_mode(str(lane_mode))
                parsed_points: list[tuple[float, float, float | None]] = []
                for pnt in points:
                    if not isinstance(pnt, (list, tuple)) or len(pnt) < 2:
                        continue
                    px = float(pnt[0])
                    py = float(pnt[1])
                    pyaw = float(pnt[2]) if len(pnt) >= 3 and pnt[2] is not None else None
                    parsed_points.append((px, py, pyaw))
                if parsed_points:
                    nav2_waypoints_by_lane[mode] = tuple(parsed_points)
        out.append(
            LaneGoalRuleV2(
                current_vertex=current_vertex,
                target_vertex=target_vertex,
                limiter_edges=tuple(limiter_edges),
                lane_goal_sign_by_lane=lane_signs,
                lane_goal_sign_by_previous=prev_signs,
                nav2_goal_by_lane=nav2_goal_by_lane,
                nav2_waypoints_by_lane=nav2_waypoints_by_lane,
                default_lane_goal_sign=-1 if int(item.get("default_lane_goal_sign", 1)) == -1 else 1,
            )
        )
    return tuple(out)
