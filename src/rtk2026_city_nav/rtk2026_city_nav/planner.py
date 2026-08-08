"""Выбор маневра в точке решения: по знаку, иначе по покрытию.

Таблица маневров предвычисляется из графа один раз и в реальном времени
только читается. Выбор из неё детерминирован при заданных состоянии, знаке
и счётчиках посещений, поэтому маршрут воспроизводим и тестируется без
робота.

Состояние маршрута — пара вершин, а не одна: метка маневра зависит от
направления прибытия, см. :mod:`rtk2026_city_nav.maneuver`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from rtk2026_city_nav.maneuver import (
    MANEUVER_ORDER,
    Candidate,
    Maneuver,
    SignAdvice,
    classify_candidates,
    tangent_at_end,
    tangent_at_start,
    turn_angle,
)
from rtk2026_city_nav.topology import Chain, Topology

#: Порог классов «прямо» и «разворот» по умолчанию.
DEFAULT_STRAIGHT_TOLERANCE_RAD = math.radians(30.0)


@dataclass(frozen=True)
class RouteState:
    """Где робот в маршруте: приехал из ``previous``, находится в ``current``."""

    previous: int
    current: int


class DecisionSource(str, Enum):
    """Почему выбран именно этот маневр. Попадает в диагностику и логи."""

    #: Распознан знак, и такой маневр доступен.
    SIGN = "sign"
    #: Знака не было либо для него нет маневра: выбор по покрытию.
    COVERAGE = "coverage"
    #: Кроме разворота выбирать было нечего.
    UTURN_FALLBACK = "uturn_fallback"


@dataclass(frozen=True)
class Decision:
    """Выбранный маневр и всё, что нужно для его исполнения и разбора."""

    state: RouteState
    #: Куда едем — следующая точка решения.
    target: int
    maneuver: Maneuver
    #: Геометрия участка, ориентированная по ходу движения.
    chain: Chain
    #: Одно из значений :class:`DecisionSource`.
    source: str
    #: Все варианты, что были на выбор. Нужны для диагностики и логов.
    candidates: tuple[Candidate, ...]
    #: Маневры, вычеркнутые запрещающим знаком.
    forbidden: frozenset[Maneuver] = frozenset()
    #: Запрет не оставил ни одного варианта и был проигнорирован. Значит
    #: знак противоречит графу — это надо видеть в диагностике.
    prohibition_ignored: bool = False

    @property
    def next_state(self) -> RouteState:
        """Состояние после того, как участок пройден."""
        return RouteState(previous=self.state.current, current=self.target)


class NoManeuverAvailable(RuntimeError):
    """В состоянии нет ни одного маневра: тупик, не предусмотренный графом."""


class ManeuverTable:
    """Предвычисленные правила движения: состояние — доступные маневры.

    Строится из топологии один раз. Для каждой пары смежных точек решений
    считаются углы поворота ко всем выходам и раскладываются по классам.
    """

    def __init__(
        self,
        topology: Topology,
        *,
        straight_tolerance_rad: float = DEFAULT_STRAIGHT_TOLERANCE_RAD,
    ) -> None:
        self._topology = topology
        self._straight_tolerance_rad = float(straight_tolerance_rad)
        self._by_state: dict[RouteState, tuple[Candidate, ...]] = {}
        self._build()

    @property
    def topology(self) -> Topology:
        return self._topology

    @property
    def states(self) -> tuple[RouteState, ...]:
        """Все состояния, для которых есть хотя бы один маневр."""
        return tuple(self._by_state)

    def candidates(self, state: RouteState) -> tuple[Candidate, ...]:
        """Маневры, доступные в состоянии; пусто, если состояние неизвестно."""
        return self._by_state.get(state, ())

    def candidate_for(self, state: RouteState, maneuver: Maneuver) -> Candidate | None:
        """Маневр заданного класса или ``None``, если такого нет."""
        for candidate in self._by_state.get(state, ()):
            if candidate.maneuver is maneuver:
                return candidate
        return None

    def chain_for(self, state: RouteState, target: int) -> Chain | None:
        """Геометрия участка от текущей вершины к цели."""
        chains = self._topology.chains_between(state.current, target)
        return chains[0] if chains else None

    def _build(self) -> None:
        for arriving in self._topology.chains:
            state = RouteState(previous=arriving.start, current=arriving.end)
            if state in self._by_state:
                # Параллельные цепочки в это состояние: угол прибытия
                # неоднозначен. Ловится проверкой единственности цепочки.
                continue

            arrival_tangent = tangent_at_end(arriving.polyline_xy)
            if arrival_tangent is None:
                continue

            turns: dict[int, float] = {}
            for outgoing in self._topology.chains_from(state.current):
                departure_tangent = tangent_at_start(outgoing.polyline_xy)
                if departure_tangent is None:
                    continue
                turns[outgoing.end] = turn_angle(arrival_tangent, departure_tangent)

            if turns:
                self._by_state[state] = classify_candidates(
                    turns, straight_tolerance_rad=self._straight_tolerance_rad
                )


@dataclass
class Planner:
    """Выбор маневра и учёт посещений.

    Знак имеет приоритет. Если знака нет или для него нет доступного
    маневра, выбирается тот, что ведёт в наименее посещённую вершину:
    маршрут по условию задачи недетерминированный, и при повторном
    попадании в ту же точку выбора робот идёт другим путём.

    Разворот исключается, пока есть что-то ещё: иначе робот застрянет,
    катаясь туда-обратно по одному участку.
    """

    table: ManeuverTable
    #: Сколько раз посещена каждая точка решения.
    visits: dict[int, int] = field(default_factory=dict[int, int])

    def decide(
        self,
        state: RouteState,
        *,
        advice: SignAdvice | None = None,
    ) -> Decision:
        """Выбрать маневр в состоянии.

        :param advice: что говорят знаки — предписание и запреты. ``None``
            означает, что знаков не было.
        :raises NoManeuverAvailable: в состоянии нет ни одного маневра.
        """
        available = self.table.candidates(state)
        if not available:
            raise NoManeuverAvailable(
                f"нет маневров в состоянии {state.previous} -> {state.current}"
            )

        hint = (advice or SignAdvice()).resolved()

        forward = tuple(c for c in available if c.maneuver is not Maneuver.UTURN)
        pool = forward if forward else available
        source = (
            DecisionSource.COVERAGE if forward else DecisionSource.UTURN_FALLBACK
        )

        # Запрет вычёркивает вариант, ничего не предлагая взамен.
        prohibition_ignored = False
        if hint.forbid:
            allowed = tuple(c for c in pool if c.maneuver not in hint.forbid)
            if allowed:
                pool = allowed
            else:
                # Запрет не оставил ничего: знак противоречит графу. Стоять
                # на месте хуже, чем проехать, поэтому запрет игнорируется,
                # но факт попадает в решение и оттуда в диагностику.
                prohibition_ignored = True

        chosen: Candidate | None = None
        if hint.prefer is not None:
            chosen = next((c for c in pool if c.maneuver is hint.prefer), None)
            if chosen is not None:
                source = DecisionSource.SIGN

        if chosen is None:
            chosen = min(pool, key=self._coverage_key)

        chain = self.table.chain_for(state, chosen.target)
        if chain is None:
            raise NoManeuverAvailable(
                f"нет цепочки {state.current} -> {chosen.target}"
            )

        return Decision(
            state=state,
            target=chosen.target,
            maneuver=chosen.maneuver,
            chain=chain,
            source=source.value,
            candidates=available,
            forbidden=hint.forbid,
            prohibition_ignored=prohibition_ignored,
        )

    def commit(self, decision: Decision) -> RouteState:
        """Отметить участок пройденным и вернуть новое состояние."""
        target = decision.target
        self.visits[target] = self.visits.get(target, 0) + 1
        return decision.next_state

    def reset_visits(self) -> None:
        self.visits.clear()

    def _coverage_key(self, candidate: Candidate) -> tuple[int, int]:
        """Наименее посещённая цель; равенство разрешается порядком маневров.

        Фиксированный порядок нужен, чтобы один и тот же прогон давал один
        и тот же маршрут — без этого поведение зависело бы от порядка
        обхода словаря.
        """
        return (
            self.visits.get(candidate.target, 0),
            MANEUVER_ORDER.index(candidate.maneuver),
        )
