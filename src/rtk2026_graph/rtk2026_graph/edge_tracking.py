"""Утилиты трекинга активного ребра для полосного движения."""

from __future__ import annotations

import math
from dataclasses import dataclass

from rtk2026_pose_graph.model import OrientedEdge, RoadGraph


@dataclass(frozen=True)
class ActiveEdgeMatch:
    """Результат выбора активного ориентированного ребра."""

    edge_id: int
    start_id: int
    end_id: int
    lateral_distance_m: float
    heading_alignment: float


def select_active_edge_from_limiters(
    graph: RoadGraph,
    limiter_edges: tuple[tuple[int, int], ...],
    pose_x: float,
    pose_y: float,
    yaw_rad: float,
) -> ActiveEdgeMatch | None:
    """Выбирает активное ребро из limiter_edges по близости и ориентации.

    Правило:
    1) ищем ребро с минимальной поперечной дистанцией до полилинии,
    2) при близких дистанциях предпочитаем ребро с лучшим совпадением heading.
    """
    candidates = _collect_candidate_edges(graph, limiter_edges)
    if not candidates:
        return None

    best: tuple[float, ActiveEdgeMatch] | None = None
    for edge in candidates:
        lateral = _distance_to_polyline(pose_x, pose_y, edge.polyline_xy)
        alignment = _heading_alignment(edge, yaw_rad)

        # Композитный скор: дистанция важнее, alignment — как tie-breaker.
        score = lateral * 10.0 + (1.0 - alignment)
        match = ActiveEdgeMatch(
            edge_id=edge.edge_id,
            start_id=edge.start_id,
            end_id=edge.end_id,
            lateral_distance_m=lateral,
            heading_alignment=alignment,
        )
        if best is None or score < best[0]:
            best = (score, match)

    assert best is not None
    return best[1]


def infer_direction_mode_from_yaw(edge: OrientedEdge, yaw_rad: float) -> str:
    """Определяет forward/reverse относительно ориентации ребра."""
    if len(edge.polyline_xy) < 2:
        return "forward"
    ax, ay = edge.polyline_xy[0]
    bx, by = edge.polyline_xy[1]
    edge_heading = math.atan2(by - ay, bx - ax)
    return "forward" if _angle_abs_diff(edge_heading, yaw_rad) <= math.pi / 2.0 else "reverse"


def _collect_candidate_edges(
    graph: RoadGraph,
    limiter_edges: tuple[tuple[int, int], ...],
) -> list[OrientedEdge]:
    out: list[OrientedEdge] = []
    for start_id, end_id in limiter_edges:
        edge = graph.edge_toward_neighbor(start_id, end_id)
        if edge is not None and len(edge.polyline_xy) >= 2:
            out.append(edge)
            continue
        # Фолбэк: если в графе задано обратное ребро, используем его как
        # виртуально ориентированное start_id -> end_id с перевернутой полилинией.
        back = graph.edge_toward_neighbor(end_id, start_id)
        if back is not None and len(back.polyline_xy) >= 2:
            out.append(
                OrientedEdge(
                    edge_id=back.edge_id,
                    start_id=start_id,
                    end_id=end_id,
                    polyline_xy=tuple(reversed(back.polyline_xy)),
                    cost=back.cost,
                    overridable=back.overridable,
                    corridor_hard_side=back.corridor_hard_side,
                )
            )
    return out


def _distance_to_polyline(px: float, py: float, polyline_xy: tuple[tuple[float, float], ...]) -> float:
    best = float("inf")
    for i in range(len(polyline_xy) - 1):
        ax, ay = polyline_xy[i]
        bx, by = polyline_xy[i + 1]
        d = math.sqrt(_point_to_segment_dist_sq(px, py, ax, ay, bx, by))
        if d < best:
            best = d
    return best if math.isfinite(best) else 0.0


def _heading_alignment(edge: OrientedEdge, yaw_rad: float) -> float:
    ax, ay = edge.polyline_xy[0]
    bx, by = edge.polyline_xy[1]
    edge_heading = math.atan2(by - ay, bx - ax)
    # 1.0 = идеально совпадает, 0.0 = противоположно.
    return max(0.0, math.cos(_angle_abs_diff(edge_heading, yaw_rad)))


def _angle_abs_diff(a: float, b: float) -> float:
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(d)


def _point_to_segment_dist_sq(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    abx = bx - ax
    aby = by - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-24:
        dx = px - ax
        dy = py - ay
        return dx * dx + dy * dy
    apx = px - ax
    apy = py - ay
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    cx = ax + t * abx
    cy = ay + t * aby
    dx = px - cx
    dy = py - cy
    return dx * dx + dy * dy
