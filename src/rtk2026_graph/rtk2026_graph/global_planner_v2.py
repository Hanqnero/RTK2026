"""Новый глобальный планер v2: выбор следующей логической вершины и переключение lane1/lane2."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from rtk2026_graph.lane_mode import normalize_lane_mode


@dataclass(frozen=True)
class SignCommandPolicy:
    preferred: tuple[str, ...] = ()
    forbidden: tuple[str, ...] = ()


@dataclass(frozen=True)
class GlobalPlannerConfigV2:
    """Конфиг глобального планера v2."""

    lane_targets: dict[str, dict[int, tuple[int, ...]]]
    intersection_vertex_ids: frozenset[int]
    lane_switch_edges: frozenset[tuple[int, int]]
    intersection_exit_lane_targets: dict[str, dict[int, tuple[int, ...]]]
    lane_direction_targets: dict[str, dict[int, dict[str, int]]]
    intersection_direction_targets: dict[int, dict[str, dict[str, int]]]
    sign_command_mapping: dict[str, SignCommandPolicy]
    initial_lane_mode: str = "lane1"


@dataclass(frozen=True)
class GlobalPlanStepV2:
    """Результат одного шага выбора цели."""

    current_vertex: int
    previous_vertex: int
    chosen_target: int
    allowed_targets: tuple[int, ...]
    active_lane_mode: str
    next_lane_mode: str
    lane_switched: bool
    pick_source: str
    route_sign_applied: bool


class GlobalPlannerV2:
    """Чистая (без ROS) логика выбора следующей вершины."""

    def __init__(self, config: GlobalPlannerConfigV2) -> None:
        lane_targets: dict[str, dict[int, tuple[int, ...]]] = {}
        for lane_mode, by_vertex in config.lane_targets.items():
            canonical = normalize_lane_mode(lane_mode)
            lane_targets[canonical] = {
                int(vertex): tuple(int(v) for v in targets) for vertex, targets in by_vertex.items()
            }
        self._config = GlobalPlannerConfigV2(
            lane_targets=lane_targets,
            intersection_vertex_ids=frozenset(int(v) for v in config.intersection_vertex_ids),
            lane_switch_edges=frozenset((int(a), int(b)) for a, b in config.lane_switch_edges),
            intersection_exit_lane_targets=self._normalize_exit_lane_targets(config.intersection_exit_lane_targets),
            lane_direction_targets=self._normalize_lane_direction_targets(config.lane_direction_targets),
            intersection_direction_targets=self._normalize_intersection_direction_targets(
                config.intersection_direction_targets
            ),
            sign_command_mapping=self._normalize_sign_command_mapping(config.sign_command_mapping),
            initial_lane_mode=normalize_lane_mode(config.initial_lane_mode),
        )

    @property
    def initial_lane_mode(self) -> str:
        return self._config.initial_lane_mode

    def allowed_targets(
        self,
        *,
        current_vertex: int,
        active_lane_mode: str,
        previous_vertex: int = -1,
        block_immediate_backtrack: bool = True,
    ) -> tuple[int, ...]:
        lane_mode = normalize_lane_mode(active_lane_mode)
        raw = self._config.lane_targets.get(lane_mode, {}).get(int(current_vertex), ())
        if not raw:
            return ()
        raw = self._filter_intersection_targets(
            current_vertex=int(current_vertex),
            previous_vertex=int(previous_vertex),
            raw_targets=raw,
        )
        if not block_immediate_backtrack:
            return raw
        if int(previous_vertex) < 0:
            return raw
        if int(previous_vertex) not in raw:
            return raw
        if len(raw) <= 1:
            return raw
        return tuple(v for v in raw if v != int(previous_vertex))

    def _filter_intersection_targets(
        self,
        *,
        current_vertex: int,
        previous_vertex: int,
        raw_targets: tuple[int, ...],
    ) -> tuple[int, ...]:
        intersection = self._config.intersection_vertex_ids
        if int(current_vertex) not in intersection:
            return raw_targets
        prev_in_intersection = int(previous_vertex) in intersection
        if prev_in_intersection:
            # exit: приехали из вершины ромба => выезжаем наружу.
            outside = tuple(v for v in raw_targets if int(v) not in intersection)
            return outside if outside else raw_targets
        # entry: въехали с внешнего контура => едем по вершинам ромба.
        inside = tuple(v for v in raw_targets if int(v) in intersection)
        return inside if inside else raw_targets

    def pick_next(
        self,
        *,
        current_vertex: int,
        active_lane_mode: str,
        previous_vertex: int = -1,
        sign_target_vertex: int = -1,
        route_sign_command: str | None = None,
        visit_counts: dict[int, int] | None = None,
        block_immediate_backtrack: bool = True,
    ) -> GlobalPlanStepV2:
        allowed = self.allowed_targets(
            current_vertex=int(current_vertex),
            active_lane_mode=active_lane_mode,
            previous_vertex=int(previous_vertex),
            block_immediate_backtrack=block_immediate_backtrack,
        )
        if not allowed:
            raise ValueError(
                f"no allowed targets for current={current_vertex}, lane={active_lane_mode}, previous={previous_vertex}"
            )

        allowed, route_sign_applied = self._apply_route_sign_command(
            current_vertex=int(current_vertex),
            previous_vertex=int(previous_vertex),
            active_lane_mode=active_lane_mode,
            allowed_targets=allowed,
            route_sign_command=route_sign_command,
        )
        if not allowed:
            raise ValueError(
                "no allowed targets after route-sign filtering for "
                f"current={current_vertex}, previous={previous_vertex}, lane={active_lane_mode}, "
                f"route_sign_command={route_sign_command}"
            )

        chosen: int
        source: str
        if int(sign_target_vertex) > 0 and int(sign_target_vertex) in allowed:
            chosen = int(sign_target_vertex)
            source = "sign"
        else:
            counts = visit_counts or {}
            chosen = min(allowed, key=lambda v: (int(counts.get(int(v), 0)), allowed.index(v)))
            source = "fallback"

        lane_mode = normalize_lane_mode(active_lane_mode)
        next_mode = self._next_lane_mode_by_intersection_exit(
            current_vertex=int(current_vertex),
            target_vertex=int(chosen),
            previous_vertex=int(previous_vertex),
            active_lane_mode=lane_mode,
        )
        switched = next_mode != lane_mode
        return GlobalPlanStepV2(
            current_vertex=int(current_vertex),
            previous_vertex=int(previous_vertex),
            chosen_target=int(chosen),
            allowed_targets=allowed,
            active_lane_mode=lane_mode,
            next_lane_mode=next_mode,
            lane_switched=switched,
            pick_source=source,
            route_sign_applied=route_sign_applied,
        )

    def _apply_route_sign_command(
        self,
        *,
        current_vertex: int,
        previous_vertex: int,
        active_lane_mode: str,
        allowed_targets: tuple[int, ...],
        route_sign_command: str | None,
    ) -> tuple[tuple[int, ...], bool]:
        if not route_sign_command:
            return allowed_targets, False
        policy = self._config.sign_command_mapping.get(str(route_sign_command).strip())
        if policy is None:
            return allowed_targets, False
        direction_targets = self._direction_targets_for_context(
            current_vertex=int(current_vertex),
            previous_vertex=int(previous_vertex),
            active_lane_mode=active_lane_mode,
        )
        if not direction_targets:
            return allowed_targets, False

        forbidden_targets = {
            int(direction_targets[name])
            for name in policy.forbidden
            if name in direction_targets and int(direction_targets[name]) in allowed_targets
        }
        filtered_allowed = tuple(v for v in allowed_targets if int(v) not in forbidden_targets)

        preferred_candidates = tuple(
            int(direction_targets[name])
            for name in policy.preferred
            if name in direction_targets and int(direction_targets[name]) in filtered_allowed
        )
        if preferred_candidates:
            return preferred_candidates, True
        if filtered_allowed != allowed_targets:
            return filtered_allowed, True
        return filtered_allowed, False

    def _direction_targets_for_context(
        self,
        *,
        current_vertex: int,
        previous_vertex: int,
        active_lane_mode: str,
    ) -> dict[str, int]:
        if int(current_vertex) in self._config.intersection_vertex_ids:
            phase = "exit" if int(previous_vertex) in self._config.intersection_vertex_ids else "entry"
            return dict(self._config.intersection_direction_targets.get(int(current_vertex), {}).get(phase, {}))
        lane_mode = normalize_lane_mode(active_lane_mode)
        return dict(self._config.lane_direction_targets.get(lane_mode, {}).get(int(current_vertex), {}))

    def _normalize_exit_lane_targets(self, raw: dict[str, dict[int, tuple[int, ...]]] | object) -> dict[str, dict[int, tuple[int, ...]]]:
        out: dict[str, dict[int, tuple[int, ...]]] = {}
        if not isinstance(raw, dict):
            return out
        for lane_mode, by_vertex in raw.items():
            if not isinstance(by_vertex, dict):
                continue
            canonical_lane = normalize_lane_mode(str(lane_mode))
            out[canonical_lane] = {}
            for vertex, targets in by_vertex.items():
                if isinstance(targets, tuple):
                    parsed = tuple(int(v) for v in targets)
                elif isinstance(targets, list):
                    parsed = tuple(int(v) for v in targets)
                else:
                    continue
                out[canonical_lane][int(vertex)] = parsed
        return out

    def _normalize_lane_direction_targets(self, raw: object) -> dict[str, dict[int, dict[str, int]]]:
        out: dict[str, dict[int, dict[str, int]]] = {}
        if not isinstance(raw, dict):
            return out
        for lane_mode, by_vertex in raw.items():
            if not isinstance(by_vertex, dict):
                continue
            canonical_lane = normalize_lane_mode(str(lane_mode))
            out[canonical_lane] = {}
            for vertex, directions in by_vertex.items():
                parsed = self._normalize_direction_mapping(directions)
                if parsed:
                    out[canonical_lane][int(vertex)] = parsed
        return out

    def _normalize_intersection_direction_targets(self, raw: object) -> dict[int, dict[str, dict[str, int]]]:
        out: dict[int, dict[str, dict[str, int]]] = {}
        if not isinstance(raw, dict):
            return out
        for vertex, by_phase in raw.items():
            if not isinstance(by_phase, dict):
                continue
            parsed_phases: dict[str, dict[str, int]] = {}
            for phase, directions in by_phase.items():
                if str(phase) not in ("entry", "exit"):
                    continue
                parsed = self._normalize_direction_mapping(directions)
                if parsed:
                    parsed_phases[str(phase)] = parsed
            if parsed_phases:
                out[int(vertex)] = parsed_phases
        return out

    def _normalize_sign_command_mapping(self, raw: object) -> dict[str, SignCommandPolicy]:
        out: dict[str, SignCommandPolicy] = {}
        if not isinstance(raw, dict):
            return out
        for command, payload in raw.items():
            if not isinstance(payload, dict):
                continue
            preferred_raw = payload.get("preferred", [])
            forbidden_raw = payload.get("forbidden", [])
            preferred = tuple(str(v).strip() for v in preferred_raw if str(v).strip())
            forbidden = tuple(str(v).strip() for v in forbidden_raw if str(v).strip())
            out[str(command).strip()] = SignCommandPolicy(preferred=preferred, forbidden=forbidden)
        return out

    def _normalize_direction_mapping(self, raw: object) -> dict[str, int]:
        out: dict[str, int] = {}
        if not isinstance(raw, dict):
            return out
        for name, target in raw.items():
            if target is None:
                continue
            try:
                out[str(name).strip()] = int(target)
            except (TypeError, ValueError):
                continue
        return out

    def _next_lane_mode_by_intersection_exit(
        self,
        *,
        current_vertex: int,
        target_vertex: int,
        previous_vertex: int,
        active_lane_mode: str,
    ) -> str:
        intersection = self._config.intersection_vertex_ids
        if int(current_vertex) not in intersection or int(previous_vertex) not in intersection:
            return normalize_lane_mode(active_lane_mode)
        if int(target_vertex) in intersection:
            return normalize_lane_mode(active_lane_mode)
        for lane_mode in ("lane1", "lane2"):
            targets = self._config.intersection_exit_lane_targets.get(lane_mode, {}).get(int(current_vertex), ())
            if int(target_vertex) in targets:
                return lane_mode
        return normalize_lane_mode(active_lane_mode)
