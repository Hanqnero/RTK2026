"""Геометрия полосы: смещение от разметочной линии в сторону движения.

Полилиния графа — разметочная линия, а не центр полосы. Робот едет сбоку
от неё, и с какой именно стороны — следствие направления шага маршрута,
а не хранимое состояние: обратный порядок пары вершин даёт касательную
с обратным знаком, а значит и нормаль с обратным.

Система координат правая, ``y`` вверх. В координатах растрового
изображения, где ``y`` вниз, все стороны получатся зеркальными.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

#: Правостороннее и левостороннее движение. Единственное место, где сторона
#: задана параметром, а не выведена из геометрии.
RIGHT_HAND_TRAFFIC = 1
LEFT_HAND_TRAFFIC = -1

_EPS = 1e-12


@dataclass(frozen=True)
class LanePose:
    """Поза в центре полосы."""

    x: float
    y: float
    #: Курс вдоль полосы, радианы.
    yaw: float

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)


def right_normal(tangent: tuple[float, float]) -> tuple[float, float]:
    """Правая нормаль к направлению движения.

    Поворот касательной на -90 градусов. Проверка: движение на восток
    ``(1, 0)`` даёт ``(0, -1)`` — на юг, куда и смотрит правая рука.
    """
    tx, ty = tangent
    return (ty, -tx)


def offset_along_chain(
    polyline_xy: tuple[tuple[float, float], ...],
    *,
    lane_offset_m: float,
    traffic_side: int = RIGHT_HAND_TRAFFIC,
    miter_limit: float = 2.0,
) -> tuple[LanePose, ...]:
    """Сместить полилинию в центр полосы.

    Полилиния должна быть уже ориентирована по ходу движения: порядок её
    точек и определяет, какая сторона правая.

    На каждую точку полилинии приходится одна поза. Во внутренних точках
    смещение идёт по биссектрисе, чтобы поза отстояла на ``lane_offset_m``
    от обеих прилегающих прямых. На остром изломе вынос по биссектрисе
    растёт неограниченно, поэтому при превышении ``miter_limit`` излом
    скругляется двумя позами вместо одной.

    :param lane_offset_m: смещение центра полосы от разметочной линии.
    :param traffic_side: :data:`RIGHT_HAND_TRAFFIC` или
        :data:`LEFT_HAND_TRAFFIC`.
    :param miter_limit: предел выноса в долях ``lane_offset_m``.
    :returns: позы в порядке движения; пусто, если полилиния короче
        двух различимых точек.
    """
    segments = _segments(polyline_xy)
    if not segments:
        return ()

    side = 1.0 if traffic_side >= 0 else -1.0
    offset = side * float(lane_offset_m)
    max_miter = abs(float(lane_offset_m)) * max(1.0, float(miter_limit))

    poses: list[LanePose] = []

    first_start, _first_end, first_tangent = segments[0]
    poses.append(_shift(first_start, first_tangent, offset))

    for (_, end, tangent), (_, next_end, next_tangent) in zip(segments, segments[1:]):
        poses.extend(
            _bend_poses(
                corner=end,
                incoming=tangent,
                outgoing=next_tangent,
                offset=offset,
                max_miter=max_miter,
            )
        )
        del next_end

    last_start, last_end, last_tangent = segments[-1]
    poses.append(_shift(last_end, last_tangent, offset))
    del last_start

    return tuple(poses)


def resample_along_chain(
    polyline_xy: tuple[tuple[float, float], ...],
    *,
    lane_offset_m: float,
    step_m: float,
    traffic_side: int = RIGHT_HAND_TRAFFIC,
    miter_limit: float = 2.0,
) -> tuple[LanePose, ...]:
    """То же смещение, но с догущением поз вдоль длинных сегментов.

    Позы во всех точках полилинии сохраняются; между ними добавляются
    промежуточные, чтобы шаг не превышал ``step_m``. Нужно, когда рёбра
    графа длиннее, чем допустимое расстояние между целями Nav2.
    """
    base = offset_along_chain(
        polyline_xy,
        lane_offset_m=lane_offset_m,
        traffic_side=traffic_side,
        miter_limit=miter_limit,
    )
    if len(base) < 2 or step_m <= 0.0:
        return base

    dense: list[LanePose] = [base[0]]

    for current, following in zip(base, base[1:]):
        span = math.hypot(following.x - current.x, following.y - current.y)
        extra = int(span / float(step_m))
        for index in range(1, extra + 1):
            ratio = index / (extra + 1)
            dense.append(
                LanePose(
                    x=current.x + (following.x - current.x) * ratio,
                    y=current.y + (following.y - current.y) * ratio,
                    yaw=current.yaw,
                )
            )
        dense.append(following)

    return tuple(dense)


def _segments(
    polyline_xy: tuple[tuple[float, float], ...],
) -> list[tuple[tuple[float, float], tuple[float, float], tuple[float, float]]]:
    """Сегменты с единичными касательными; вырожденные отбрасываются."""
    out: list[tuple[tuple[float, float], tuple[float, float], tuple[float, float]]] = []

    for start, end in zip(polyline_xy, polyline_xy[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < _EPS:
            continue
        out.append((start, end, (dx / length, dy / length)))

    return out


def _shift(
    point: tuple[float, float],
    tangent: tuple[float, float],
    offset: float,
) -> LanePose:
    nx, ny = right_normal(tangent)
    return LanePose(
        x=point[0] + nx * offset,
        y=point[1] + ny * offset,
        yaw=math.atan2(tangent[1], tangent[0]),
    )


def _bend_poses(
    *,
    corner: tuple[float, float],
    incoming: tuple[float, float],
    outgoing: tuple[float, float],
    offset: float,
    max_miter: float,
) -> list[LanePose]:
    """Позы в изломе: одна по биссектрисе либо две при остром угле."""
    n_in = right_normal(incoming)
    n_out = right_normal(outgoing)

    bx, by = n_in[0] + n_out[0], n_in[1] + n_out[1]
    bisector_length = math.hypot(bx, by)

    if bisector_length >= _EPS:
        # Проекция биссектрисы на нормаль равна косинусу половины угла
        # излома, поэтому вынос компенсирует его именно так.
        projection = (bx / bisector_length) * n_in[0] + (by / bisector_length) * n_in[1]
        if abs(projection) >= _EPS:
            miter = abs(offset) / abs(projection)
            if miter <= max_miter:
                scale = offset / projection
                return [
                    LanePose(
                        x=corner[0] + (bx / bisector_length) * scale,
                        y=corner[1] + (by / bisector_length) * scale,
                        yaw=math.atan2(outgoing[1], outgoing[0]),
                    )
                ]

    # Излом слишком острый: биссектриса выбросила бы позу далеко за полосу.
    # Скругляем - конец входящего сегмента и начало исходящего.
    return [_shift(corner, incoming, offset), _shift(corner, outgoing, offset)]
