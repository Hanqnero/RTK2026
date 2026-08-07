"""Коридор ребра: с какой стороны непересекаемая граница (вариант B).

С одной стороны ребра — непересекаемая граница (парапет, стена),
с другой — обычная зона costmap, где препятствия объезжаются.

Сторона читается из ``metadata`` ребра по ключу ``corridor_hard_side``.
"""

from __future__ import annotations

from typing import Literal

from rtk2026_pose_graph.model import OrientedEdge

#: Сторона «жёсткой» границы относительно направления движения по ребру.
CorridorHardSide = Literal["left", "right"]

#: Основной ключ metadata и его исторический синоним: в файлах графа
#: встречаются оба написания.
METADATA_KEY = "corridor_hard_side"
METADATA_KEY_LEGACY = "hard_side"


def edge_hard_side(edge: OrientedEdge) -> CorridorHardSide | None:
    """Сторона жёсткой границы ребра или ``None``, если не задана.

    Неизвестные значения трактуются как отсутствие границы.
    """
    raw = edge.meta(METADATA_KEY, edge.meta(METADATA_KEY_LEGACY))
    if raw is None:
        return None

    normalized = str(raw).strip().lower()
    if normalized == "left":
        return "left"
    if normalized == "right":
        return "right"
    return None


def violates_hard_corridor(
    lateral_m: float,
    hard_side: CorridorHardSide | None,
    tol_m: float,
) -> bool:
    """Вышла ли точка за допустимую полуплоскость коридора.

    ``lateral_m`` — знаковое поперечное смещение из
    :func:`rtk2026_pose_graph.polyline_signed_lateral_m`: положительное
    слева от направления движения.

    * ``hard_side == "left"`` — граница слева: нельзя уходить влево дальше ``tol_m``.
    * ``hard_side == "right"`` — граница справа: нельзя уходить вправо дальше ``tol_m``.
    * ``None`` — ограничения нет.
    """
    if hard_side == "left":
        return lateral_m > tol_m
    if hard_side == "right":
        return lateral_m < -tol_m
    return False
