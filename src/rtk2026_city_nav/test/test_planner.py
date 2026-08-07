import math

import pytest

from rtk2026_city_nav.maneuver import Maneuver
from rtk2026_city_nav.planner import (
    DecisionSource,
    ManeuverTable,
    NoManeuverAvailable,
    Planner,
    RouteState,
)
from rtk2026_city_nav.topology import build_topology
from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph

TOL = math.radians(30.0)


def _graph(
    nodes: dict[int, tuple[float, float]],
    edges: list[tuple[int, int, int]],
) -> RoadGraph:
    return RoadGraph(
        nodes={
            nid: Node(node_id=nid, x=xy[0], y=xy[1]) for nid, xy in nodes.items()
        },
        edges={
            eid: OrientedEdge(
                edge_id=eid, start_id=a, end_id=b, polyline_xy=(nodes[a], nodes[b])
            )
            for eid, a, b in edges
        },
    )


def _cross_table() -> ManeuverTable:
    """Перекрёсток в центре с четырьмя тупиковыми лучами.

        20 (север)
         |
    40 --0-- 10   (запад, центр, восток)
         |
        30 (юг)
    """
    graph = _graph(
        nodes={
            0: (0.0, 0.0),
            10: (1.0, 0.0),
            20: (0.0, 1.0),
            30: (0.0, -1.0),
            40: (-1.0, 0.0),
        },
        edges=[(1, 0, 10), (2, 0, 20), (3, 0, 30), (4, 0, 40)],
    )
    return ManeuverTable(build_topology(graph), straight_tolerance_rad=TOL)


def _from_south() -> RouteState:
    """Приехал с юга в центр: еду на север."""
    return RouteState(previous=30, current=0)


def test_table_lists_all_four_maneuvers_from_south() -> None:
    table = _cross_table()

    by_maneuver = {
        c.maneuver: c.target for c in table.candidates(_from_south())
    }

    assert by_maneuver == {
        Maneuver.STRAIGHT: 20,
        Maneuver.RIGHT: 10,
        Maneuver.LEFT: 40,
        Maneuver.UTURN: 30,
    }


def test_table_relabels_arms_for_a_different_arrival() -> None:
    table = _cross_table()

    from_east = {
        c.maneuver: c.target
        for c in table.candidates(RouteState(previous=10, current=0))
    }

    # Приехал с востока, еду на запад: север стал правым.
    assert from_east[Maneuver.STRAIGHT] == 40
    assert from_east[Maneuver.RIGHT] == 20
    assert from_east[Maneuver.LEFT] == 30


def test_table_has_no_candidates_for_unknown_state() -> None:
    table = _cross_table()

    assert table.candidates(RouteState(previous=999, current=0)) == ()


def test_candidate_for_finds_by_class() -> None:
    table = _cross_table()

    right = table.candidate_for(_from_south(), Maneuver.RIGHT)
    assert right is not None and right.target == 10

    assert table.candidate_for(RouteState(previous=999, current=0), Maneuver.LEFT) is None


def test_sign_wins_when_available() -> None:
    planner = Planner(_cross_table())

    decision = planner.decide(_from_south(), sign=Maneuver.LEFT)

    assert decision.target == 40
    assert decision.maneuver is Maneuver.LEFT
    assert decision.source == DecisionSource.SIGN


def test_uturn_is_never_taken_by_sign_when_forward_exists() -> None:
    planner = Planner(_cross_table())

    # Разворот исключён из пула, пока есть что-то ещё.
    decision = planner.decide(_from_south(), sign=Maneuver.UTURN)

    assert decision.maneuver is not Maneuver.UTURN
    assert decision.source == DecisionSource.COVERAGE


def test_without_sign_picks_least_visited() -> None:
    planner = Planner(_cross_table())
    state = _from_south()

    # Север уже посещали, восток и запад нет.
    planner.visits[20] = 2

    decision = planner.decide(state)

    assert decision.target != 20
    assert decision.source == DecisionSource.COVERAGE


def test_equal_counts_resolved_by_fixed_maneuver_order() -> None:
    planner = Planner(_cross_table())
    state = _from_south()

    # Счётчики равны: порядок маневров ставит straight первым.
    first = planner.decide(state)
    assert first.maneuver is Maneuver.STRAIGHT

    # Тот же прогон даёт тот же результат: выбор не зависит от порядка словаря.
    assert planner.decide(state).target == first.target


def test_repeated_visits_rotate_the_choice() -> None:
    planner = Planner(_cross_table())
    state = _from_south()

    chosen: list[int] = []
    for _ in range(3):
        decision = planner.decide(state)
        chosen.append(decision.target)
        planner.commit(decision)

    # Каждый раз другой выход: покрытие расходится по трём направлениям.
    assert len(set(chosen)) == 3
    assert 30 not in chosen  # разворот не берётся


def test_commit_increments_visits_and_shifts_state() -> None:
    planner = Planner(_cross_table())
    state = _from_south()

    decision = planner.decide(state)
    following = planner.commit(decision)

    assert planner.visits[decision.target] == 1
    assert following == RouteState(previous=0, current=decision.target)
    assert following == decision.next_state


def test_sign_without_available_maneuver_falls_back_to_coverage() -> None:
    """Т-образный: прямо ехать некуда, знак «прямо» игнорируется."""
    graph = _graph(
        nodes={0: (0.0, 0.0), 10: (1.0, 0.0), 30: (0.0, -1.0), 40: (-1.0, 0.0)},
        edges=[(1, 0, 10), (2, 0, 30), (3, 0, 40)],
    )
    planner = Planner(ManeuverTable(build_topology(graph), straight_tolerance_rad=TOL))
    state = RouteState(previous=30, current=0)

    assert planner.table.candidate_for(state, Maneuver.STRAIGHT) is None

    decision = planner.decide(state, sign=Maneuver.STRAIGHT)

    assert decision.source == DecisionSource.COVERAGE
    assert decision.maneuver in (Maneuver.LEFT, Maneuver.RIGHT)


def test_dead_end_falls_back_to_uturn() -> None:
    """Тупиковый отросток: кроме разворота выбирать нечего."""
    graph = _graph(
        nodes={0: (0.0, 0.0), 1: (1.0, 0.0), 2: (2.0, 0.0)},
        edges=[(1, 0, 1), (2, 1, 2)],
    )
    topology = build_topology(graph)
    planner = Planner(ManeuverTable(topology, straight_tolerance_rad=TOL))

    # Приехал в тупик 2 из 0.
    decision = planner.decide(RouteState(previous=0, current=2))

    assert decision.maneuver is Maneuver.UTURN
    assert decision.source == DecisionSource.UTURN_FALLBACK


def test_unknown_state_raises() -> None:
    planner = Planner(_cross_table())

    with pytest.raises(NoManeuverAvailable):
        planner.decide(RouteState(previous=999, current=888))


def test_decision_carries_chain_oriented_along_travel() -> None:
    planner = Planner(_cross_table())

    decision = planner.decide(_from_south(), sign=Maneuver.RIGHT)

    assert decision.chain.start == 0
    assert decision.chain.end == 10
    # Полилиния идёт от текущей вершины к цели.
    assert decision.chain.polyline_xy[0] == (0.0, 0.0)
    assert decision.chain.polyline_xy[-1] == (1.0, 0.0)


def test_decision_keeps_all_candidates_for_diagnostics() -> None:
    planner = Planner(_cross_table())

    decision = planner.decide(_from_south(), sign=Maneuver.RIGHT)

    # В решении видно не только что выбрано, но и из чего выбирали.
    assert {c.maneuver for c in decision.candidates} == set(Maneuver)


def test_reset_visits_clears_coverage_history() -> None:
    planner = Planner(_cross_table())
    planner.visits[20] = 5

    planner.reset_visits()

    assert planner.visits == {}
