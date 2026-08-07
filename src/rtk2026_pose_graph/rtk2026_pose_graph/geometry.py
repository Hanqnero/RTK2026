"""Геометрия полилинии ребра: проекция точки, длина, движение вдоль пути.

Только общие операции над геометрией графа. Никаких правил движения:
что делать со знаковым смещением или пройденной дугой, решает алгоритм,
а не этот модуль.

Все функции работают с полилинией как с последовательностью точек
``((x, y), ...)`` в порядке от начала ребра к концу. Направление обхода
задаёт знак поперечного смещения и отсчёт длины дуги.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PolylineProjection:
    """Проекция точки на полилинию — всё, что даёт один проход по сегментам.

    Отдельные функции ниже — обёртки над этим результатом. Считать проекцию
    один раз и брать нужные поля дешевле, чем вызывать их по очереди.
    """

    #: Координаты ближайшей точки на полилинии.
    x: float
    y: float

    #: Расстояние до полилинии, всегда неотрицательное.
    distance_m: float

    #: Знаковое поперечное смещение относительно бесконечной прямой
    #: ближайшего сегмента. Положительное — слева от направления обхода
    #: (правило правой руки в XY).
    signed_lateral_m: float

    #: Длина вдоль полилинии от её начала до проекции.
    arc_length_m: float

    #: Индекс сегмента, на который спроецировалась точка.
    segment_index: int

    #: Направление этого сегмента, радианы.
    heading_rad: float


def project_point_on_polyline(
    px: float,
    py: float,
    polyline_xy: tuple[tuple[float, float], ...],
) -> PolylineProjection | None:
    """Спроецировать точку на полилинию.

    Ближайший сегмент выбирается по расстоянию до самого сегмента (с учётом
    его концов), а знаковое смещение считается относительно бесконечной
    прямой этого сегмента: у сегмента есть сторона даже там, где сама
    проекция упёрлась в его конец.

    :returns: ``None``, если полилиния короче двух точек.
    """
    if len(polyline_xy) < 2:
        return None

    best: PolylineProjection | None = None
    arc_before_segment = 0.0

    for index in range(len(polyline_xy) - 1):
        ax, ay = polyline_xy[index]
        bx, by = polyline_xy[index + 1]

        abx, aby = bx - ax, by - ay
        segment_length = math.hypot(abx, aby)

        if segment_length < _EPS_LENGTH:
            # Вырожденный сегмент: направления нет, пропускаем его как
            # кандидата, но длину дуги не сдвигаем.
            continue

        t = _clamp01(((px - ax) * abx + (py - ay) * aby) / (segment_length * segment_length))
        cx, cy = ax + t * abx, ay + t * aby
        distance = math.hypot(px - cx, py - cy)

        if best is None or distance < best.distance_m:
            cross_z = abx * (py - ay) - aby * (px - ax)
            best = PolylineProjection(
                x=cx,
                y=cy,
                distance_m=distance,
                signed_lateral_m=cross_z / segment_length,
                arc_length_m=arc_before_segment + t * segment_length,
                segment_index=index,
                heading_rad=math.atan2(aby, abx),
            )

        arc_before_segment += segment_length

    return best


def polyline_signed_lateral_m(
    px: float,
    py: float,
    polyline_xy: tuple[tuple[float, float], ...],
) -> float:
    """Знаковое поперечное смещение точки до полилинии, метры.

    Положительное значение — слева от направления обхода. Для полилинии
    короче двух точек возвращает ``0.0``: стороны у неё нет.
    """
    projection = project_point_on_polyline(px, py, polyline_xy)
    return 0.0 if projection is None else projection.signed_lateral_m


def point_to_polyline_distance_m(
    px: float,
    py: float,
    polyline_xy: tuple[tuple[float, float], ...],
) -> float:
    """Расстояние от точки до полилинии, метры. Для вырожденной — ``0.0``."""
    projection = project_point_on_polyline(px, py, polyline_xy)
    return 0.0 if projection is None else projection.distance_m


def polyline_length_m(polyline_xy: tuple[tuple[float, float], ...]) -> float:
    """Полная длина полилинии, метры."""
    total = 0.0
    for index in range(len(polyline_xy) - 1):
        ax, ay = polyline_xy[index]
        bx, by = polyline_xy[index + 1]
        total += math.hypot(bx - ax, by - ay)
    return total


def interpolate_along_polyline(
    polyline_xy: tuple[tuple[float, float], ...],
    arc_length_m: float,
) -> tuple[float, float, float] | None:
    """Точка на заданной длине дуги от начала полилинии.

    Нужна, чтобы расставлять цели и проверки вдоль ребра: «через 1.5 м
    от начала». Значения за пределами полилинии зажимаются к её концам,
    поэтому функция всегда даёт точку на пути.

    :returns: ``(x, y, heading_rad)`` или ``None`` для полилинии короче
        двух точек.
    """
    if len(polyline_xy) < 2:
        return None

    remaining = max(0.0, arc_length_m)
    last_valid: tuple[float, float, float] | None = None

    for index in range(len(polyline_xy) - 1):
        ax, ay = polyline_xy[index]
        bx, by = polyline_xy[index + 1]

        abx, aby = bx - ax, by - ay
        segment_length = math.hypot(abx, aby)

        if segment_length < _EPS_LENGTH:
            continue

        heading = math.atan2(aby, abx)
        last_valid = (bx, by, heading)

        if remaining <= segment_length:
            t = remaining / segment_length
            return (ax + t * abx, ay + t * aby, heading)

        remaining -= segment_length

    # Запрошенная длина больше самой полилинии: возвращаем её конец.
    return last_valid


_EPS_LENGTH = 1e-12


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))
