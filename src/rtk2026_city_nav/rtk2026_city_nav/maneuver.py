"""Классификация маневра в точке решения: налево, направо, прямо, разворот.

Метка принадлежит не вершине, а паре «откуда приехал и где находишься»:
одни и те же лучи перекрёстка получают разные метки при разном направлении
прибытия. Поэтому классификация всегда требует касательной прибытия.

Касательные берутся у самой вершины — последний сегмент входящей цепочки
и первый сегмент исходящей, — а не как прямая между точками решений:
на изогнутой цепочке эта прямая уже не параллельна её концам.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

_EPS = 1e-12


class Maneuver(str, Enum):
    """Класс маневра. Значения совпадают с именами в данных знаков."""

    LEFT = "left"
    RIGHT = "right"
    STRAIGHT = "straight"
    UTURN = "uturn"


#: Порядок разрешения равенства при выборе. Фиксирован, чтобы один и тот же
#: прогон давал один и тот же маршрут.
MANEUVER_ORDER: tuple[Maneuver, ...] = (
    Maneuver.STRAIGHT,
    Maneuver.RIGHT,
    Maneuver.LEFT,
    Maneuver.UTURN,
)


@dataclass(frozen=True)
class Candidate:
    """Возможный маневр из точки решения."""

    #: Куда ведёт.
    target: int
    #: Знаковый угол между прибытием и отбытием, радианы.
    turn_rad: float
    maneuver: Maneuver

    @property
    def turn_deg(self) -> float:
        return math.degrees(self.turn_rad)


@dataclass(frozen=True)
class SignAdvice:
    """Что дорожные знаки говорят о выборе маневра.

    Знаки бывают двух родов, и это разные воздействия. Предписывающий
    указывает маневр, запрещающий лишь вычёркивает вариант, ничего не
    предлагая взамен — после него выбор идёт обычным порядком из того,
    что осталось.
    """

    #: Предписанный маневр либо ``None``.
    prefer: Maneuver | None = None
    #: Запрещённые маневры.
    forbid: frozenset[Maneuver] = frozenset()

    @property
    def is_empty(self) -> bool:
        return self.prefer is None and not self.forbid

    def resolved(self) -> SignAdvice:
        """Снять предписание, если оно же и запрещено.

        Противоречие возможно: два знака могли попасть в кадр вместе либо
        один из них распознан неверно. Запрет сильнее — нарушить запрет
        опаснее, чем не выполнить предписание.
        """
        if self.prefer is not None and self.prefer in self.forbid:
            return SignAdvice(prefer=None, forbid=self.forbid)
        return self


def turn_angle(
    arrival_tangent: tuple[float, float],
    departure_tangent: tuple[float, float],
) -> float:
    """Знаковый угол поворота, радианы, в диапазоне (-pi, pi].

    Положительный угол — против часовой, то есть налево (при ``y`` вверх).

    Считается через ``atan2`` от косого и скалярного произведений. Косого
    одного недостаточно: оно близко к нулю и при движении прямо, и при
    развороте, а различает их знак скалярного.
    """
    ax, ay = arrival_tangent
    bx, by = departure_tangent
    cross = ax * by - ay * bx
    dot = ax * bx + ay * by
    return math.atan2(cross, dot)


def classify(turn_rad: float, *, straight_tolerance_rad: float) -> Maneuver:
    """Отнести угол к классу по абсолютной величине.

    Порога недостаточно на косом перекрёстке, где два выхода попадают
    в один класс: там результат уточняет :func:`classify_candidates`.
    """
    magnitude = abs(turn_rad)

    if magnitude <= straight_tolerance_rad:
        return Maneuver.STRAIGHT
    if magnitude >= math.pi - straight_tolerance_rad:
        return Maneuver.UTURN
    return Maneuver.LEFT if turn_rad > 0.0 else Maneuver.RIGHT


def classify_candidates(
    turns: dict[int, float],
    *,
    straight_tolerance_rad: float,
) -> tuple[Candidate, ...]:
    """Разложить все выходы точки решения по классам.

    ``turns`` — угол поворота для каждой вершины-кандидата.

    Сначала работает порог. Если в один класс попало больше одного
    кандидата — они разделяются по порядку угла: самый отрицательный
    становится ``right``, самый положительный ``left``, ближайший к нулю
    ``straight``. Без этого на перекрёстке с выходами под -20 и +20
    градусов оба назывались бы ``straight`` и различить их было бы нечем.

    :returns: кандидаты в порядке возрастания угла.
    """
    if not turns:
        return ()

    ordered = sorted(turns.items(), key=lambda item: (item[1], item[0]))
    classified = {
        target: classify(turn, straight_tolerance_rad=straight_tolerance_rad)
        for target, turn in ordered
    }

    for maneuver in (Maneuver.STRAIGHT, Maneuver.LEFT, Maneuver.RIGHT, Maneuver.UTURN):
        clashing = [target for target, value in classified.items() if value == maneuver]
        if len(clashing) > 1:
            classified.update(_split_by_order(clashing, turns))

    return tuple(
        Candidate(target=target, turn_rad=turn, maneuver=classified[target])
        for target, turn in ordered
    )


def _split_by_order(
    targets: list[int],
    turns: dict[int, float],
) -> dict[int, Maneuver]:
    """Развести кандидатов одного класса по крайним и среднему.

    Больше трёх кандидатов в одном классе развести некуда: у нас всего три
    содержательных метки. Лишние остаются с меткой ближайшего края, и это
    поднимет проверку однозначности при загрузке графа.
    """
    ranked = sorted(targets, key=lambda target: turns[target])

    out: dict[int, Maneuver] = {}
    out[ranked[0]] = Maneuver.RIGHT
    out[ranked[-1]] = Maneuver.LEFT

    middle = ranked[1:-1]
    if middle:
        nearest_zero = min(middle, key=lambda target: abs(turns[target]))
        out[nearest_zero] = Maneuver.STRAIGHT
        for target in middle:
            if target not in out:
                # Развести некуда: пометим краем, ближе к которому угол.
                out[target] = (
                    Maneuver.RIGHT if turns[target] < 0.0 else Maneuver.LEFT
                )

    return out


def tangent_at_end(polyline_xy: tuple[tuple[float, float], ...]) -> tuple[float, float] | None:
    """Касательная последнего невырожденного сегмента — направление прибытия."""
    for start, end in zip(reversed(polyline_xy[:-1]), reversed(polyline_xy[1:])):
        tangent = _unit(start, end)
        if tangent is not None:
            return tangent
    return None


def tangent_at_start(polyline_xy: tuple[tuple[float, float], ...]) -> tuple[float, float] | None:
    """Касательная первого невырожденного сегмента — направление отбытия."""
    for start, end in zip(polyline_xy, polyline_xy[1:]):
        tangent = _unit(start, end)
        if tangent is not None:
            return tangent
    return None


def _unit(
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float] | None:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < _EPS:
        return None
    return (dx / length, dy / length)
