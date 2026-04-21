"""Чистая геометрия цели Nav2 со смещением в полосу (без ROS)."""

from __future__ import annotations

import math


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def project_goal_on_lane(
    vertex_x: float,
    vertex_y: float,
    edge_polyline: tuple[tuple[float, float], ...],
    *,
    lane_right_offset_m: float,
    lane_half_width_m: float,
    lane_safety_margin_m: float,
    lane_goal_offset_sign: int = 1,
) -> tuple[float, float, float, float]:
    """Смещение цели от вершины вдоль нормали к последнему сегменту полилинии.

    Направление «вдоль полосы» задаётся последним сегментом полилинии: от предпоследней точки к последней
    (в limiter_edges это ребро от начальной вершины id к конечной id у цели).
    Касательная (tx, ty); «вправо» — нормаль (ty, -tx); lane_goal_offset_sign +1 — смещение вправо, -1 — влево.

    Возвращает (goal_x, goal_y, yaw_rad, clamped_offset_m).
    При короткой полилинии (<2 точек) — цель в вершине, yaw=0, offset только для логов.
    """
    goal_x = float(vertex_x)
    goal_y = float(vertex_y)
    max_center_offset = max(0.0, lane_half_width_m - lane_safety_margin_m)
    sign = -1 if lane_goal_offset_sign == -1 else 1
    effective = float(lane_right_offset_m) * sign
    clamped_offset = clamp(effective, -max_center_offset, max_center_offset)
    yaw = 0.0

    if len(edge_polyline) >= 2:
        ax, ay = edge_polyline[-2]
        bx, by = edge_polyline[-1]
        dx = bx - ax
        dy = by - ay
        seg_len = (dx * dx + dy * dy) ** 0.5
        if seg_len > 1e-9:
            tx = dx / seg_len
            ty = dy / seg_len
            rx = ty
            ry = -tx
            goal_x = goal_x + rx * clamped_offset
            goal_y = goal_y + ry * clamped_offset
            yaw = math.atan2(ty, tx)

    return (goal_x, goal_y, yaw, clamped_offset)
