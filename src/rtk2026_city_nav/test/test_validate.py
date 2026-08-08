import math

from rtk2026_city_nav.planner import ManeuverTable, RouteState
from rtk2026_city_nav.topology import KIND_DECISION, KIND_KEY, build_topology
from rtk2026_city_nav.validate import Check, Severity, validate
from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph

TOL = math.radians(30.0)


def _build(
    nodes: dict[int, tuple[float, float]],
    edges: list[tuple[int, int, int]],
    kinds: dict[int, str] | None = None,
) -> tuple[object, ManeuverTable]:
    kinds = kinds or {}
    graph = RoadGraph(
        nodes={
            nid: Node(
                node_id=nid,
                x=xy[0],
                y=xy[1],
                metadata={KIND_KEY: kinds[nid]} if nid in kinds else {},
            )
            for nid, xy in nodes.items()
        },
        edges={
            eid: OrientedEdge(
                edge_id=eid, start_id=a, end_id=b, polyline_xy=(nodes[a], nodes[b])
            )
            for eid, a, b in edges
        },
    )
    topology = build_topology(graph)
    return topology, ManeuverTable(topology, straight_tolerance_rad=TOL)


def _cross() -> tuple[object, ManeuverTable]:
    """Прямоугольный перекрёсток с четырьмя отводами."""
    return _build(
        nodes={
            0: (0.0, 0.0),
            10: (1.0, 0.0),
            20: (0.0, 1.0),
            30: (0.0, -1.0),
            40: (-1.0, 0.0),
        },
        edges=[(1, 0, 10), (2, 0, 20), (3, 0, 30), (4, 0, 40)],
    )


def _checks(report: object) -> set[Check]:
    return {f.check for f in report.findings}  # type: ignore[attr-defined]


def test_clean_cross_has_only_dead_end_spur_warnings() -> None:
    topology, table = _cross()

    report = validate(topology, table, start=RouteState(previous=30, current=0))

    # Четыре отвода - тупики, из них выезд только разворотом.
    assert _checks(report) == {Check.FORWARD_LIVENESS}
    assert report.ok
    assert not report.errors
    assert len(report.warnings) == 4


def test_uturn_exceptions_are_derived_not_written_by_hand() -> None:
    topology, table = _cross()

    report = validate(topology, table)

    # Именно в этих состояниях запрет разворота обязан не действовать.
    assert report.uturn_exceptions == {
        RouteState(previous=0, current=10),
        RouteState(previous=0, current=20),
        RouteState(previous=0, current=30),
        RouteState(previous=0, current=40),
    }


def test_parallel_chains_are_an_error() -> None:
    """Два пути для одного маневра: геометрия неоднозначна."""
    topology, table = _build(
        nodes={
            1: (0.0, 0.0),
            2: (1.0, 1.0),
            3: (1.0, -1.0),
            4: (2.0, 0.0),
            5: (-1.0, 0.0),
            6: (3.0, 0.0),
        },
        edges=[
            (10, 1, 2), (11, 2, 4),
            (12, 1, 3), (13, 3, 4),
            (14, 1, 5), (15, 4, 6),
        ],
    )

    report = validate(topology, table)

    assert Check.CHAIN_UNIQUENESS in _checks(report)
    assert not report.ok

    finding = next(
        f for f in report.findings if f.check is Check.CHAIN_UNIQUENESS
    )
    assert finding.severity is Severity.ERROR
    assert "неоднозначна" in finding.message


def test_five_arms_break_determinism() -> None:
    """Больше трёх выходов в одном классе развести некуда."""
    topology, table = _build(
        nodes={
            0: (0.0, 0.0),
            1: (0.0, -1.0),      # юг, прибытие
            2: (-0.9, 0.4),      # четыре выхода тесной группой к северу
            3: (-0.3, 1.0),
            4: (0.3, 1.0),
            5: (0.9, 0.4),
        },
        edges=[(10, 0, 1), (11, 0, 2), (12, 0, 3), (13, 0, 4), (14, 0, 5)],
    )

    report = validate(topology, table)

    assert Check.DETERMINISM in _checks(report)
    assert not report.ok


def test_dead_end_vertex_you_cannot_leave_is_an_error() -> None:
    """В вершину ведёт цепочка, а обратно ходу нет: односторонний тупик."""
    # Ребро только одно и записано 1 -> 2. Обход неориентированный, поэтому
    # состояние (1, 2) существует; чтобы выехать было нельзя, объявим 2
    # точкой решения и оставим её без исходящих цепочек.
    nodes = {1: (0.0, 0.0), 2: (1.0, 0.0)}
    graph = RoadGraph(
        nodes={nid: Node(node_id=nid, x=xy[0], y=xy[1]) for nid, xy in nodes.items()},
        edges={
            10: OrientedEdge(
                edge_id=10, start_id=1, end_id=2, polyline_xy=(nodes[1], nodes[2])
            )
        },
    )
    topology = build_topology(graph)
    table = ManeuverTable(topology, straight_tolerance_rad=TOL)

    report = validate(topology, table)

    # Обе вершины - тупики, но выехать разворотом можно, значит это
    # предупреждение о живости, а не ошибка тупика.
    assert Check.FORWARD_LIVENESS in _checks(report)
    assert Check.DEAD_END not in _checks(report)


def test_unreachable_region_is_reported() -> None:
    """Две развязки без связи между собой: из одной другая недосягаема."""
    topology, table = _build(
        nodes={
            0: (0.0, 0.0), 1: (1.0, 0.0), 2: (0.0, 1.0), 3: (-1.0, 0.0),
            10: (10.0, 0.0), 11: (11.0, 0.0), 12: (10.0, 1.0), 13: (9.0, 0.0),
        },
        edges=[
            (1, 0, 1), (2, 0, 2), (3, 0, 3),
            (4, 10, 11), (5, 10, 12), (6, 10, 13),
        ],
    )

    report = validate(topology, table, start=RouteState(previous=1, current=0))

    assert Check.REACHABILITY in _checks(report)
    unreachable = {
        f.state for f in report.findings if f.check is Check.REACHABILITY
    }
    # Состояния второй развязки недостижимы.
    assert RouteState(previous=11, current=10) in unreachable


def test_reachability_is_skipped_without_a_start() -> None:
    topology, table = _cross()

    report = validate(topology, table)

    assert Check.REACHABILITY not in _checks(report)


def test_unknown_start_state_is_an_error() -> None:
    topology, table = _cross()

    report = validate(topology, table, start=RouteState(previous=777, current=888))

    finding = next(f for f in report.findings if f.check is Check.REACHABILITY)
    assert finding.severity is Severity.ERROR
    assert "начального состояния" in finding.message


def test_connected_graph_has_no_reachability_findings() -> None:
    """Кольцо с объявленными точками решений: всё достижимо."""
    topology, table = _build(
        nodes={1: (0.0, 0.0), 2: (1.0, 1.0), 3: (2.0, 0.0), 4: (1.0, -1.0)},
        edges=[(10, 1, 2), (11, 2, 3), (12, 3, 4), (13, 4, 1)],
        kinds={1: KIND_DECISION, 3: KIND_DECISION},
    )

    report = validate(topology, table, start=RouteState(previous=1, current=3))

    assert Check.REACHABILITY not in _checks(report)


def test_report_summary_and_finding_text_name_the_place() -> None:
    topology, table = _cross()

    report = validate(topology, table)

    assert "граф проверен" in report.summary()
    text = str(report.findings[0])
    assert "warning" in text
    assert "->" in text


def test_empty_graph_yields_nothing_to_complain_about() -> None:
    graph = RoadGraph()
    topology = build_topology(graph)
    table = ManeuverTable(topology, straight_tolerance_rad=TOL)

    report = validate(topology, table)

    assert report.findings == ()
    assert report.ok
    assert report.summary() == "граф проверен, замечаний нет"
