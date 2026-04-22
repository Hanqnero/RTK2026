"""Новый локальный планер v2: постановка цели Nav2 по паре (current -> target)."""

from __future__ import annotations

from dataclasses import dataclass
import math

from rtk2026_graph.lane_goal_geometry import project_goal_on_lane
from rtk2026_graph.lane_mode import normalize_lane_mode
from rtk2026_graph.model import RoadGraph


@dataclass(frozen=True)
class LaneGoalRuleV2:
    """Геометрические правила для одной пары (current -> target)."""

    current_vertex: int
    target_vertex: int
    limiter_edges: tuple[tuple[int, int], ...]


@dataclass(frozen=True)
class LocalGoalV2:
    """Готовая цель для Nav2."""

    x: float
    y: float
    yaw: float
    clamped_offset: float
    limiter_edges: tuple[tuple[int, int], ...]
    lane_goal_sign: int
    reference_polyline: tuple[tuple[float, float], ...]
    waypoint_index: int
    waypoint_count: int


class LocalPlannerV2:
    """Чистая (без ROS) геометрия постановки цели в полосу."""

    def __init__(self, graph: RoadGraph, rules: tuple[LaneGoalRuleV2, ...]) -> None:
        self._graph = graph
        self._rules = tuple(rules)

    def build_goal(
        self,
        *,
        current_vertex: int,
        target_vertex: int,
        lane_mode: str,
        previous_vertex: int = -1,
        waypoint_index: int = 0,
        lane_right_offset_m: float = 0.20,
        lane_half_width_m: float = 0.25,
        lane_safety_margin_m: float = 0.03,
    ) -> LocalGoalV2:
        rule = self._find_rule(int(current_vertex), int(target_vertex))
        if rule is None:
            # Упрощенный режим: локальные правила в конфиге необязательны.
            # Если явного правила нет, используем текущее ребро current->target
            # и базовую схему полос.
            rule = self._default_rule(int(current_vertex), int(target_vertex))
        polyline = self._polyline_for_rule(rule, int(target_vertex))
        if polyline is None:
            raise ValueError(f"no polyline for ({current_vertex} -> {target_vertex})")
        node = self._graph.nodes.get(int(target_vertex))
        if node is None:
            raise ValueError(f"target vertex not found in graph: {target_vertex}")
        normalized_lane = normalize_lane_mode(lane_mode)
        sign = 1
        if normalized_lane == "lane2":
            reference_polyline = self._offset_polyline_right(polyline, abs(float(lane_right_offset_m)))
        else:
            reference_polyline = polyline
        _, _, _, offset = project_goal_on_lane(
            node.x,
            node.y,
            polyline,
            lane_right_offset_m=float(lane_right_offset_m),
            lane_half_width_m=float(lane_half_width_m),
            lane_safety_margin_m=float(lane_safety_margin_m),
            lane_goal_offset_sign=int(sign),
        )
        fallback_goal = project_goal_on_lane(
            node.x,
            node.y,
            polyline,
            lane_right_offset_m=float(lane_right_offset_m),
            lane_half_width_m=float(lane_half_width_m),
            lane_safety_margin_m=float(lane_safety_margin_m),
            lane_goal_offset_sign=int(sign),
        )
        waypoints = self._goal_waypoints_for_lane(
            rule=rule,
            lane_mode=normalized_lane,
            polyline=polyline,
            fallback_goal=(float(fallback_goal[0]), float(fallback_goal[1]), float(fallback_goal[2])),
        )
        idx = min(max(0, int(waypoint_index)), max(0, len(waypoints) - 1))
        gx, gy, yaw = waypoints[idx]
        return LocalGoalV2(
            x=float(gx),
            y=float(gy),
            yaw=float(yaw),
            clamped_offset=float(offset),
            limiter_edges=rule.limiter_edges,
            lane_goal_sign=int(sign),
            reference_polyline=tuple(reference_polyline),
            waypoint_index=int(idx),
            waypoint_count=int(len(waypoints)),
        )

    def _find_rule(self, current_vertex: int, target_vertex: int) -> LaneGoalRuleV2 | None:
        for rule in self._rules:
            if int(rule.current_vertex) == int(current_vertex) and int(rule.target_vertex) == int(target_vertex):
                return rule
        return None

    def _default_rule(self, current_vertex: int, target_vertex: int) -> LaneGoalRuleV2:
        return LaneGoalRuleV2(
            current_vertex=int(current_vertex),
            target_vertex=int(target_vertex),
            limiter_edges=((int(current_vertex), int(target_vertex)),),
        )

    def _polyline_for_rule(
        self,
        rule: LaneGoalRuleV2,
        target_vertex: int,
    ) -> tuple[tuple[float, float], ...] | None:
        three_vertices_curve = self._curve_from_two_edges_vertices(rule.limiter_edges)
        if three_vertices_curve is not None:
            return three_vertices_curve
        chained = self._chain_limiter_polylines(rule.limiter_edges)
        if chained is not None:
            return self._smooth_polyline(chained)
        poly = self._edge_polyline_or_reversed(int(rule.current_vertex), int(target_vertex))
        if poly is not None:
            return self._smooth_polyline(poly)
        return None

    def _edge_polyline_or_reversed(
        self,
        start_id: int,
        end_id: int,
    ) -> tuple[tuple[float, float], ...] | None:
        edge = self._graph.edge_toward_neighbor(int(start_id), int(end_id))
        if edge is not None and edge.polyline_xy:
            return tuple(edge.polyline_xy)
        back = self._graph.edge_toward_neighbor(int(end_id), int(start_id))
        if back is not None and back.polyline_xy:
            return tuple(reversed(back.polyline_xy))
        return None

    def _chain_limiter_polylines(
        self,
        limiter_edges: tuple[tuple[int, int], ...],
    ) -> tuple[tuple[float, float], ...] | None:
        if not limiter_edges:
            return None
        merged: list[tuple[float, float]] = []
        for idx, (start_id, end_id) in enumerate(limiter_edges):
            poly = self._edge_polyline_or_reversed(int(start_id), int(end_id))
            if poly is None or len(poly) < 2:
                return None
            if idx == 0:
                merged.extend(poly)
                continue
            prev_x, prev_y = merged[-1]
            cur_x, cur_y = poly[0]
            if abs(prev_x - cur_x) < 1e-6 and abs(prev_y - cur_y) < 1e-6:
                merged.extend(poly[1:])
            else:
                merged.extend(poly)
        return tuple(merged) if len(merged) >= 2 else None

    def _smooth_polyline(
        self,
        polyline: tuple[tuple[float, float], ...],
    ) -> tuple[tuple[float, float], ...]:
        if len(polyline) < 3:
            return polyline
        dense = self._densify_polyline(polyline, max_step_m=0.12)
        return self._catmull_rom_spline(dense, samples_per_segment=4)

    def _curve_from_two_edges_vertices(
        self,
        limiter_edges: tuple[tuple[int, int], ...],
    ) -> tuple[tuple[float, float], ...] | None:
        """Для двух связанных limiter-ребер строим гладкую линию по 3 вершинам.

        Идея: достаточно точек start/joint/end, где joint — общая вершина ребер.
        """
        if len(limiter_edges) != 2:
            return None
        a0, a1 = limiter_edges[0]
        b0, b1 = limiter_edges[1]
        if int(a1) != int(b0):
            return None
        n0 = self._graph.nodes.get(int(a0))
        n1 = self._graph.nodes.get(int(a1))
        n2 = self._graph.nodes.get(int(b1))
        if n0 is None or n1 is None or n2 is None:
            return None
        p0 = (float(n0.x), float(n0.y))
        p1 = (float(n1.x), float(n1.y))
        p2 = (float(n2.x), float(n2.y))
        return self._quadratic_bezier(p0, p1, p2, samples=24)

    def _quadratic_bezier(
        self,
        p0: tuple[float, float],
        p1: tuple[float, float],
        p2: tuple[float, float],
        *,
        samples: int,
    ) -> tuple[tuple[float, float], ...]:
        count = max(2, int(samples))
        out: list[tuple[float, float]] = []
        for i in range(count):
            t = float(i) / float(count - 1)
            omt = 1.0 - t
            x = (omt * omt * p0[0]) + (2.0 * omt * t * p1[0]) + (t * t * p2[0])
            y = (omt * omt * p0[1]) + (2.0 * omt * t * p1[1]) + (t * t * p2[1])
            out.append((float(x), float(y)))
        return tuple(out)

    def _offset_polyline_right(
        self,
        polyline: tuple[tuple[float, float], ...],
        offset_m: float,
    ) -> tuple[tuple[float, float], ...]:
        if len(polyline) < 2 or offset_m <= 1e-9:
            return polyline
        out: list[tuple[float, float]] = []
        last_tangent: tuple[float, float] = (1.0, 0.0)
        for i in range(len(polyline)):
            px, py = polyline[i]
            if i == 0:
                x0, y0 = polyline[i]
                x1, y1 = polyline[i + 1]
                tx, ty = (x1 - x0), (y1 - y0)
            elif i == len(polyline) - 1:
                x0, y0 = polyline[i - 1]
                x1, y1 = polyline[i]
                tx, ty = (x1 - x0), (y1 - y0)
            else:
                xb, yb = polyline[i - 1]
                xf, yf = polyline[i + 1]
                tx, ty = (xf - xb), (yf - yb)
            norm = math.hypot(tx, ty)
            if norm > 1e-9:
                tx, ty = tx / norm, ty / norm
                last_tangent = (tx, ty)
            else:
                tx, ty = last_tangent
            rx, ry = ty, -tx
            out.append((float(px + rx * offset_m), float(py + ry * offset_m)))
        return tuple(out)

    def _goal_waypoints_for_lane(
        self,
        *,
        rule: LaneGoalRuleV2,
        lane_mode: str,
        polyline: tuple[tuple[float, float], ...],
        fallback_goal: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], ...]:
        return (fallback_goal,)

    def _polyline_terminal_yaw(self, polyline: tuple[tuple[float, float], ...]) -> float:
        if len(polyline) < 2:
            return 0.0
        ax, ay = polyline[-2]
        bx, by = polyline[-1]
        return float(math.atan2(by - ay, bx - ax))

    def _densify_polyline(
        self,
        polyline: tuple[tuple[float, float], ...],
        *,
        max_step_m: float,
    ) -> tuple[tuple[float, float], ...]:
        if len(polyline) < 2 or max_step_m <= 1e-6:
            return polyline
        out: list[tuple[float, float]] = [polyline[0]]
        for i in range(1, len(polyline)):
            x0, y0 = polyline[i - 1]
            x1, y1 = polyline[i]
            dx = x1 - x0
            dy = y1 - y0
            seg = math.hypot(dx, dy)
            steps = max(1, int(math.ceil(seg / max_step_m)))
            for j in range(1, steps + 1):
                t = float(j) / float(steps)
                out.append((x0 + dx * t, y0 + dy * t))
        return tuple(out)

    def _catmull_rom_spline(
        self,
        polyline: tuple[tuple[float, float], ...],
        *,
        samples_per_segment: int,
    ) -> tuple[tuple[float, float], ...]:
        n = len(polyline)
        if n < 3 or samples_per_segment < 1:
            return polyline
        out: list[tuple[float, float]] = [polyline[0]]
        for i in range(n - 1):
            p0 = polyline[i - 1] if i > 0 else polyline[i]
            p1 = polyline[i]
            p2 = polyline[i + 1]
            p3 = polyline[i + 2] if (i + 2) < n else polyline[i + 1]
            for j in range(1, samples_per_segment + 1):
                t = float(j) / float(samples_per_segment)
                t2 = t * t
                t3 = t2 * t
                x = 0.5 * (
                    (2.0 * p1[0])
                    + (-p0[0] + p2[0]) * t
                    + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                    + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3
                )
                y = 0.5 * (
                    (2.0 * p1[1])
                    + (-p0[1] + p2[1]) * t
                    + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                    + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3
                )
                out.append((float(x), float(y)))
        # Последняя точка должна совпасть с конечной вершиной трассы.
        out[-1] = polyline[-1]
        return tuple(out)
