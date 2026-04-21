"""Чистый локальный планер v3: последовательность Nav2-точек для пары вершин."""

from __future__ import annotations

from dataclasses import dataclass
import math

from rtk2026_graph.lane_mode import normalize_lane_mode
from rtk2026_graph.local_planner_v2 import LaneGoalRuleV2
from rtk2026_graph.model import RoadGraph


@dataclass(frozen=True)
class LocalGoalPointV3:
    x: float
    y: float
    yaw: float
    lane_goal_sign: int
    waypoint_index: int
    waypoint_count: int
    limiter_edges: tuple[tuple[int, int], ...]


class LocalPlannerPointsV3:
    """Минимальная геометрия: midpoint/угол/end + manual fallback из YAML."""

    _RIGHT_SIDE_SIGN = 1

    def __init__(self, graph: RoadGraph, rules: tuple[LaneGoalRuleV2, ...]) -> None:
        self._graph = graph
        self._rules = tuple(rules)
        self._default_offset_m = 0.20
        self._auto_cache: dict[tuple[int, int, int], tuple[tuple[float, float, float], ...]] = {}
        self._warmup()

    def build_goal_sequence(
        self,
        *,
        current_vertex: int,
        target_vertex: int,
        lane_mode: str,
        previous_vertex: int = -1,
        lane_right_offset_m: float = 0.20,
    ) -> tuple[LocalGoalPointV3, ...]:
        rule = self._find_rule(int(current_vertex), int(target_vertex))
        if rule is None:
            raise ValueError(f"no local rule for ({current_vertex} -> {target_vertex})")

        sign = self._lane_offset_sign()
        offset_m = abs(float(lane_right_offset_m))

        pts: tuple[tuple[float, float, float], ...] = ()
        if abs(offset_m - self._default_offset_m) <= 1e-6:
            pts = self._auto_cache.get((int(rule.current_vertex), int(rule.target_vertex), sign), ())
        if not pts:
            pts = self._auto_points(
                rule=rule,
                current_vertex=int(current_vertex),
                target_vertex=int(target_vertex),
                sign=sign,
                offset_m=offset_m,
            )
        if not pts:
            raise ValueError(
                f"no auto-generated nav2 points for ({current_vertex} -> {target_vertex}); "
                "check limiter_edges orientation and graph nodes"
            )

        count = len(pts)
        return tuple(
            LocalGoalPointV3(
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                lane_goal_sign=int(sign),
                waypoint_index=i,
                waypoint_count=count,
                limiter_edges=tuple(rule.limiter_edges),
            )
            for i, (x, y, yaw) in enumerate(pts)
        )

    def _warmup(self) -> None:
        for rule in self._rules:
            cur = int(rule.current_vertex)
            tgt = int(rule.target_vertex)
            sign = self._lane_offset_sign()
            auto = self._auto_points(
                rule=rule,
                current_vertex=cur,
                target_vertex=tgt,
                sign=sign,
                offset_m=self._default_offset_m,
            )
            if auto:
                self._auto_cache[(cur, tgt, sign)] = auto

    def _lane_offset_sign(self) -> int:
        # Рабочая полоса всегда справа относительно ориентированной цепочки
        # limiter_edges/current->target; lane1/lane2 влияет только на выбор
        # логического маршрута, а не на сторону локального оффсета.
        return self._RIGHT_SIDE_SIGN

    def _find_rule(self, current_vertex: int, target_vertex: int) -> LaneGoalRuleV2 | None:
        for rule in self._rules:
            if int(rule.current_vertex) == current_vertex and int(rule.target_vertex) == target_vertex:
                return rule
        return None

    def _auto_points(
        self,
        *,
        rule: LaneGoalRuleV2,
        current_vertex: int,
        target_vertex: int,
        sign: int,
        offset_m: float,
    ) -> tuple[tuple[float, float, float], ...]:
        edges = tuple(rule.limiter_edges)
        if not edges:
            return ()
        if len(edges) == 1:
            s, e = self._orient_single_edge(edges[0], current_vertex=current_vertex, target_vertex=target_vertex)
            p1 = self._shifted_on_edge(s, e, t=0.50, sign=sign, offset_m=offset_m)
            p2 = self._shifted_on_edge(s, e, t=1.00, sign=sign, offset_m=offset_m)
            return tuple(p for p in (p1, p2) if p is not None)

        e1, e2 = self._order_turn_edges(edges[0], edges[1], current_vertex=current_vertex, target_vertex=target_vertex)
        p1 = self._shifted_on_edge(e1[0], e1[1], t=0.50, sign=sign, offset_m=offset_m)
        p2 = self._shifted_on_edge(e2[0], e2[1], t=0.50, sign=sign, offset_m=offset_m)
        p3 = self._shifted_on_edge(e2[0], e2[1], t=1.00, sign=sign, offset_m=offset_m)
        if self._is_collinear_transition(e1, e2):
            # Для прямых участков 2-реберной цепочки не добавляем "угловую" точку:
            # только сдвинутые середины ребер + финал сегмента.
            return tuple(p for p in (p1, p2, p3) if p is not None)
        pc = self._shifted_corner(e1, e2, sign=sign, offset_m=offset_m)
        return tuple(p for p in (p1, pc, p2, p3) if p is not None)

    def _orient_single_edge(
        self,
        edge: tuple[int, int],
        *,
        current_vertex: int,
        target_vertex: int,
    ) -> tuple[int, int]:
        a, b = int(edge[0]), int(edge[1])
        direct = int(a == current_vertex) + int(b == target_vertex)
        rev = int(b == current_vertex) + int(a == target_vertex)
        return (b, a) if rev > direct else (a, b)

    def _order_turn_edges(
        self,
        edge_a: tuple[int, int],
        edge_b: tuple[int, int],
        *,
        current_vertex: int,
        target_vertex: int,
    ) -> tuple[tuple[int, int], tuple[int, int]]:
        a = (int(edge_a[0]), int(edge_a[1]))
        b = (int(edge_b[0]), int(edge_b[1]))
        cands = (
            (a, b), (a, (b[1], b[0])), ((a[1], a[0]), b), ((a[1], a[0]), (b[1], b[0])),
            (b, a), (b, (a[1], a[0])), ((b[1], b[0]), a), ((b[1], b[0]), (a[1], a[0])),
        )
        best = (a, b)
        best_score = -10_000
        for e1, e2 in cands:
            if e1[1] != e2[0]:
                continue
            score = 10 * int(e1[0] == current_vertex) + 10 * int(e2[1] == target_vertex)
            if score > best_score:
                best_score = score
                best = (e1, e2)
        return best

    def _shifted_on_edge(
        self,
        start_id: int,
        end_id: int,
        *,
        t: float,
        sign: int,
        offset_m: float,
    ) -> tuple[float, float, float] | None:
        s = self._graph.nodes.get(int(start_id))
        e = self._graph.nodes.get(int(end_id))
        if s is None or e is None:
            return None
        vx = float(e.x) - float(s.x)
        vy = float(e.y) - float(s.y)
        n = math.hypot(vx, vy)
        if n <= 1e-9:
            return None
        tx, ty = vx / n, vy / n
        rx, ry = ty, -tx
        side = -1.0 if int(sign) == -1 else 1.0
        tc = max(0.0, min(1.0, float(t)))
        cx = float(s.x) + vx * tc
        cy = float(s.y) + vy * tc
        return (cx + rx * side * offset_m, cy + ry * side * offset_m, math.atan2(ty, tx))

    def _shifted_corner(
        self,
        e1: tuple[int, int],
        e2: tuple[int, int],
        *,
        sign: int,
        offset_m: float,
    ) -> tuple[float, float, float] | None:
        a, b = int(e1[0]), int(e1[1])
        b0, c = int(e2[0]), int(e2[1])
        if b != b0:
            return None
        na = self._graph.nodes.get(a)
        nb = self._graph.nodes.get(b)
        nc = self._graph.nodes.get(c)
        if na is None or nb is None or nc is None:
            return None
        v1x, v1y = float(nb.x) - float(na.x), float(nb.y) - float(na.y)
        v2x, v2y = float(nc.x) - float(nb.x), float(nc.y) - float(nb.y)
        n1, n2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
        if n1 <= 1e-9 or n2 <= 1e-9:
            return None
        t1x, t1y = v1x / n1, v1y / n1
        t2x, t2y = v2x / n2, v2y / n2
        r1x, r1y = t1y, -t1x
        r2x, r2y = t2y, -t2x
        bx, by = r1x + r2x, r1y + r2y
        bn = math.hypot(bx, by)
        if bn <= 1e-9:
            bx, by = r2x, r2y
            bn = math.hypot(bx, by)
            if bn <= 1e-9:
                return None
        bx, by = bx / bn, by / bn
        side = -1.0 if int(sign) == -1 else 1.0
        return (
            float(nb.x) + bx * side * offset_m,
            float(nb.y) + by * side * offset_m,
            math.atan2(t2y, t2x),
        )

    def _is_collinear_transition(self, e1: tuple[int, int], e2: tuple[int, int]) -> bool:
        a = self._graph.nodes.get(int(e1[0]))
        b = self._graph.nodes.get(int(e1[1]))
        c = self._graph.nodes.get(int(e2[1]))
        if a is None or b is None or c is None:
            return False
        v1x = float(b.x) - float(a.x)
        v1y = float(b.y) - float(a.y)
        v2x = float(c.x) - float(b.x)
        v2y = float(c.y) - float(b.y)
        n1 = math.hypot(v1x, v1y)
        n2 = math.hypot(v2x, v2y)
        if n1 <= 1e-9 or n2 <= 1e-9:
            return False
        t1x, t1y = v1x / n1, v1y / n1
        t2x, t2y = v2x / n2, v2y / n2
        dot = t1x * t2x + t1y * t2y
        cross = t1x * t2y - t1y * t2x
        return dot > 0.995 and abs(cross) <= 0.05
