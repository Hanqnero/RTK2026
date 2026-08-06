"""Геометрия варианта B: полилиния ребра, знаковое смещение, жёсткая сторона коридора."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rtk2026_pose_graph.model import CorridorHardSide


def polyline_signed_lateral_m(
    px: float,
    py: float,
    polyline_xy: tuple[tuple[float, float], ...],
) -> float:
    """Знаковое поперечное смещение точки (px, py) до ближайшего сегмента полилинии (метры).

    Направление обхода — от первой вершины к последней. Знак: положительное значение
    соответствует полуплоскости *слева* от вектора сегмента (правило правой руки в XY).
    """
    if len(polyline_xy) < 2:
        return 0.0

    best: tuple[float, float] | None = None
    for i in range(len(polyline_xy) - 1):
        ax, ay = polyline_xy[i]
        bx, by = polyline_xy[i + 1]
        lat = _signed_lateral_to_segment_m(px, py, ax, ay, bx, by)
        d_sq = _point_to_segment_perp_dist_sq(px, py, ax, ay, bx, by)
        if best is None or d_sq < best[0]:
            best = (d_sq, lat)

    assert best is not None
    return best[1]


def violates_hard_corridor(
    lateral_m: float,
    hard_side: CorridorHardSide | None,
    tol_m: float,
) -> bool:
    """True, если точка выходит за допустимую полуплоскость относительно «жёсткой» стороны.

    * ``hard_side == "left"`` — граница слева от движения: запрет уходить влево дальше ``tol_m``.
    * ``hard_side == "right"`` — граница справа: запрет уходить вправо дальше ``tol_m``.
    """
    if hard_side is None:
        return False
    if hard_side == "left":
        return lateral_m > tol_m
    if hard_side == "right":
        return lateral_m < -tol_m
    return False


def _signed_lateral_to_segment_m(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    abx = bx - ax
    aby = by - ay
    ab_len = math.hypot(abx, aby)
    if ab_len < 1e-12:
        return 0.0
    cross_z = abx * (py - ay) - aby * (px - ax)
    return cross_z / ab_len


def _point_to_segment_perp_dist_sq(
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
