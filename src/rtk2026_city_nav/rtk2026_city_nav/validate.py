"""Проверки графа при загрузке.

Все пять условий проверяются на предвычисленной таблице маневров, до
выезда. Нарушение любого — дефект графа или его разметки, а не ошибка
рантайма: в движении оно проявится как непредсказуемый выбор или как
застрявший робот, и разбираться будет уже некогда.

Проверки ничего не бросают. Вызывающий сам решает, что делать с находкой:
одни дефекты делают запуск бессмысленным, другие лишь сужают маршрут.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from rtk2026_city_nav.maneuver import Maneuver
from rtk2026_city_nav.planner import (
    DEFAULT_MAX_TURN_RAD,
    ManeuverTable,
    RouteState,
    reachable,
)
from rtk2026_city_nav.topology import Topology


class Severity(str, Enum):
    """Насколько находка мешает движению."""

    #: Ехать нельзя либо поведение непредсказуемо.
    ERROR = "error"
    #: Ехать можно, но маршрут ограничен или требует оговорки.
    WARNING = "warning"


class Check(str, Enum):
    """Какое условие нарушено."""

    #: Одному маневру в состоянии соответствует больше одного выхода.
    DETERMINISM = "determinism"
    #: Из состояния нет ни одного маневра.
    DEAD_END = "dead_end"
    #: Из состояния нет ничего, кроме разворота.
    FORWARD_LIVENESS = "forward_liveness"
    #: Не все состояния достижимы из начального.
    REACHABILITY = "reachability"
    #: Между смежными вершинами больше одной цепочки.
    CHAIN_UNIQUENESS = "chain_uniqueness"


@dataclass(frozen=True)
class Finding:
    """Одна находка проверки."""

    check: Check
    severity: Severity
    message: str
    #: Состояние, к которому относится находка, если применимо.
    state: RouteState | None = None

    def __str__(self) -> str:
        where = (
            f" [{self.state.previous} -> {self.state.current}]"
            if self.state is not None
            else ""
        )
        return f"{self.severity.value}: {self.check.value}{where}: {self.message}"


@dataclass(frozen=True)
class Report:
    """Итог всех проверок."""

    findings: tuple[Finding, ...]

    @property
    def errors(self) -> tuple[Finding, ...]:
        return tuple(f for f in self.findings if f.severity is Severity.ERROR)

    @property
    def warnings(self) -> tuple[Finding, ...]:
        return tuple(f for f in self.findings if f.severity is Severity.WARNING)

    @property
    def ok(self) -> bool:
        """Нет ошибок. Предупреждения движению не мешают."""
        return not self.errors

    #: Состояния, где разворот обязан быть разрешён, иначе робот встанет.
    #: Выводится из проверки живости, а не пишется руками.
    @property
    def uturn_exceptions(self) -> frozenset[RouteState]:
        return frozenset(
            f.state
            for f in self.findings
            if f.check is Check.FORWARD_LIVENESS and f.state is not None
        )

    def summary(self) -> str:
        if not self.findings:
            return "граф проверен, замечаний нет"
        return (
            f"граф проверен: ошибок {len(self.errors)}, "
            f"предупреждений {len(self.warnings)}"
        )


def validate(
    topology: Topology,
    table: ManeuverTable,
    *,
    start: RouteState | None = None,
    max_turn_rad: float = DEFAULT_MAX_TURN_RAD,
) -> Report:
    """Прогнать все проверки.

    :param start: начальное состояние. Без него проверка достижимости
        пропускается: не от чего считать.
    :param max_turn_rad: предел крутизны маневра, тот же, что у выбора.
    """
    findings: list[Finding] = []

    findings.extend(_check_chain_uniqueness(topology))
    findings.extend(_check_determinism(table, max_turn_rad))
    findings.extend(_check_dead_ends(topology, table))
    findings.extend(_check_forward_liveness(table, max_turn_rad))

    if start is not None:
        findings.extend(_check_reachability(table, start, max_turn_rad))

    return Report(findings=tuple(findings))


def _check_chain_uniqueness(topology: Topology) -> list[Finding]:
    """Между смежными вершинами должна быть ровно одна цепочка.

    Несколько путей для одного маневра означают, что его геометрия
    неоднозначна и выбор пути произволен. Лечится удалением избыточных
    внутренних рёбер графа.
    """
    seen: dict[tuple[int, int], int] = {}
    for chain in topology.chains:
        key = (chain.start, chain.end)
        seen[key] = seen.get(key, 0) + 1

    return [
        Finding(
            check=Check.CHAIN_UNIQUENESS,
            severity=Severity.ERROR,
            message=(
                f"между {start} и {end} цепочек {count}: геометрия маневра "
                "неоднозначна, выбор пути произволен"
            ),
            state=RouteState(previous=start, current=end),
        )
        for (start, end), count in sorted(seen.items())
        if count > 1
    ]


def _check_determinism(table: ManeuverTable, max_turn_rad: float) -> list[Finding]:
    """Одному маневру в состоянии — не больше одного выхода.

    Нарушается, когда несколько цепочек уходят из вершины по одному и тому
    же ребру: касательная отбытия у них общая, значит и метка. Так бывает,
    когда развилка стоит дальше по ходу, в геометрической вершине.

    Это предупреждение, а не ошибка. Ехать не мешает: выбор по покрытию
    берёт наименее посещённую цель, а исполнитель ведёт робота по всей
    цепочке целиком. Мешает только знаку — предписание в таком состоянии
    неразличимо, и выбор между одинаково помеченными выходами окажется
    произвольным. Знать, где это так, надо; запрещать выезд — нет.
    """
    findings: list[Finding] = []

    for state in table.states:
        by_maneuver: dict[Maneuver, list[int]] = {}
        for candidate in reachable(
            table.candidates(state), state, max_turn_rad=max_turn_rad
        ):
            by_maneuver.setdefault(candidate.maneuver, []).append(candidate.target)

        for maneuver, targets in sorted(by_maneuver.items()):
            if len(targets) > 1:
                findings.append(
                    Finding(
                        check=Check.DETERMINISM,
                        severity=Severity.WARNING,
                        message=(
                            f"маневр {maneuver.value} ведёт сразу в {sorted(targets)}: "
                            "знак здесь выберет произвольно, по покрытию поедет верно"
                        ),
                        state=state,
                    )
                )

    return findings


def _check_dead_ends(topology: Topology, table: ManeuverTable) -> list[Finding]:
    """Из каждого достижимого состояния должен быть хоть один маневр.

    Состояние возникает, когда в вершину ведёт цепочка. Если при этом
    из вершины не выходит ничего, робот, доехав, встанет.
    """
    findings: list[Finding] = []

    for chain in topology.chains:
        state = RouteState(previous=chain.start, current=chain.end)
        if not table.candidates(state):
            findings.append(
                Finding(
                    check=Check.DEAD_END,
                    severity=Severity.ERROR,
                    message=(
                        f"в вершину {chain.end} можно приехать, но выехать нельзя"
                    ),
                    state=state,
                )
            )

    return findings


def _check_forward_liveness(table: ManeuverTable, max_turn_rad: float) -> list[Finding]:
    """Из каждого состояния должен быть маневр, отличный от разворота.

    Где его нет — настоящий тупиковый отросток. Ехать оттуда можно только
    разворотом, и запрет немедленного разворота там обязан не действовать.
    Это предупреждение, а не ошибка: тупик бывает частью трассы.
    """
    findings: list[Finding] = []

    for state in table.states:
        forward = reachable(
            table.candidates(state), state, max_turn_rad=max_turn_rad
        )
        if not forward:
            findings.append(
                Finding(
                    check=Check.FORWARD_LIVENESS,
                    severity=Severity.WARNING,
                    message=(
                        f"из вершины {state.current} вперёд хода нет: "
                        "тупик, выезд только разворотом"
                    ),
                    state=state,
                )
            )

    return findings


def _check_reachability(
    table: ManeuverTable, start: RouteState, max_turn_rad: float
) -> list[Finding]:
    """Все состояния должны быть достижимы из начального.

    Иначе исследование запирает себя в подобласти: счётчики посещений
    растут, а часть трассы остаётся недосягаемой. Разворот в обходе
    учитывается — без него достижимость была бы занижена.
    """
    if not table.candidates(start) and start not in table.states:
        return [
            Finding(
                check=Check.REACHABILITY,
                severity=Severity.ERROR,
                message="начального состояния нет в таблице маневров",
                state=start,
            )
        ]

    reached = {start}
    frontier = [start]
    while frontier:
        state = frontier.pop()
        available = table.candidates(state)
        # Из тупика выбор идёт по недоступному, иначе ехать было бы некуда.
        # Обход повторяет это, чтобы не объявлять недостижимым то, куда
        # робот доедет.
        pool = reachable(available, state, max_turn_rad=max_turn_rad) or available
        for candidate in pool:
            following = RouteState(previous=state.current, current=candidate.target)
            if following not in reached:
                reached.add(following)
                frontier.append(following)

    unreachable = sorted(
        (s for s in table.states if s not in reached),
        key=lambda s: (s.previous, s.current),
    )

    return [
        Finding(
            check=Check.REACHABILITY,
            severity=Severity.WARNING,
            message=(
                f"состояние недостижимо из старта "
                f"{start.previous} -> {start.current}"
            ),
            state=state,
        )
        for state in unreachable
    ]
