"""Утилиты трекинга активного ребра для полосного движения."""

from __future__ import annotations

import math
from dataclasses import dataclass

from rtk2026_pose_graph.geometry import point_to_polyline_distance_m
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
        lateral = point_to_polyline_distance_m(pose_x, pose_y, edge.polyline_xy)
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
        #
        # Аннотации копируются без изменений, как и было до выноса модели
        # графа. Для corridor_hard_side это под вопросом: при развороте
        # направления сторона границы меняется на противоположную, а здесь
        # переносится как есть. Поведение сохранено намеренно, чтобы вынос
        # модели не менял работу алгоритма; разобраться стоит при его
        # переписывании.
        back = graph.edge_toward_neighbor(end_id, start_id)
        if back is not None and len(back.polyline_xy) >= 2:
            out.append(back.reversed())
    return out


def _heading_alignment(edge: OrientedEdge, yaw_rad: float) -> float:
    ax, ay = edge.polyline_xy[0]
    bx, by = edge.polyline_xy[1]
    edge_heading = math.atan2(by - ay, bx - ax)
    # 1.0 = идеально совпадает, 0.0 = противоположно.
    return max(0.0, math.cos(_angle_abs_diff(edge_heading, yaw_rad)))


def _angle_abs_diff(a: float, b: float) -> float:
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return abs(d)
