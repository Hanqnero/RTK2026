"""Загрузка конфигурации v2-планеров из YAML."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from rtk2026_graph.global_planner_v2 import GlobalPlannerConfigV2
from rtk2026_graph.local_planner_v2 import LaneGoalRuleV2
from rtk2026_graph.lane_mode import normalize_lane_mode


def load_planner_v2_config_path(
    path: str | Path,
    *,
    sign_direction_topology_path: str | Path | None = None,
) -> tuple[GlobalPlannerConfigV2, tuple[LaneGoalRuleV2, ...]]:
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
    if sign_direction_topology_path is not None:
        sign_path = Path(sign_direction_topology_path)
        if not sign_path.is_absolute():
            sign_path = (p.parent / sign_path).resolve()
        with sign_path.open(encoding="utf-8") as sf:
            sign_data = yaml.safe_load(sf) or {}
        if not isinstance(sign_data, dict):
            raise ValueError("sign direction topology: YAML root must be map")
        data.update(sign_data)
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
        lane_direction_targets=_parse_lane_direction_targets(data.get("sign_direction_topology", {})),
        intersection_direction_targets=_parse_intersection_direction_targets(data.get("sign_direction_topology", {})),
        sign_command_mapping=_parse_sign_command_mapping(data.get("sign_command_mapping", {})),
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


def _parse_lane_direction_targets(raw: Any) -> dict[str, dict[int, dict[str, int]]]:
    if not isinstance(raw, dict):
        return {}
    lane_vertices = raw.get("lane_vertices", {})
    if not isinstance(lane_vertices, dict):
        return {}
    out: dict[str, dict[int, dict[str, int]]] = {}
    for lane_mode, by_vertex in lane_vertices.items():
        if not isinstance(by_vertex, dict):
            continue
        canonical_lane = normalize_lane_mode(str(lane_mode))
        out[canonical_lane] = {}
        for vertex, directions in by_vertex.items():
            parsed = _parse_direction_mapping(directions)
            if parsed:
                out[canonical_lane][int(vertex)] = parsed
    return out


def _parse_intersection_direction_targets(raw: Any) -> dict[int, dict[str, dict[str, int]]]:
    if not isinstance(raw, dict):
        return {}
    intersection_vertices = raw.get("intersection_vertices", {})
    if not isinstance(intersection_vertices, dict):
        return {}
    out: dict[int, dict[str, dict[str, int]]] = {}
    for vertex, by_phase in intersection_vertices.items():
        if not isinstance(by_phase, dict):
            continue
        parsed_phases: dict[str, dict[str, int]] = {}
        for phase, directions in by_phase.items():
            if str(phase) not in ("entry", "exit"):
                continue
            parsed = _parse_direction_mapping(directions)
            if parsed:
                parsed_phases[str(phase)] = parsed
        if parsed_phases:
            out[int(vertex)] = parsed_phases
    return out


def _parse_sign_command_mapping(raw: Any) -> dict[str, dict[str, tuple[str, ...]]]:
    if not isinstance(raw, dict):
        return {}
    out: dict[str, dict[str, tuple[str, ...]]] = {}
    for command, payload in raw.items():
        if not isinstance(payload, dict):
            continue
        preferred = tuple(str(v).strip() for v in payload.get("preferred", []) if str(v).strip())
        forbidden = tuple(str(v).strip() for v in payload.get("forbidden", []) if str(v).strip())
        out[str(command).strip()] = {
            "preferred": preferred,
            "forbidden": forbidden,
        }
    return out


def _parse_direction_mapping(raw: Any) -> dict[str, int]:
    if not isinstance(raw, dict):
        return {}
    out: dict[str, int] = {}
    for name, target in raw.items():
        if target is None:
            continue
        try:
            out[str(name).strip()] = int(target)
        except (TypeError, ValueError):
            continue
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
        out.append(
            LaneGoalRuleV2(
                current_vertex=current_vertex,
                target_vertex=target_vertex,
                limiter_edges=tuple(limiter_edges),
            )
        )
    return tuple(out)
