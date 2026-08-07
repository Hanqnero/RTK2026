from rtk2026_city_nav.topology import (
    KIND_DECISION,
    KIND_KEY,
    KIND_PASSTHROUGH,
    build_topology,
    undirected_degrees,
)
from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph


def _graph(
    nodes: dict[int, tuple[float, float]],
    edges: list[tuple[int, int, int]],
    kinds: dict[int, str] | None = None,
) -> RoadGraph:
    """Граф из координат и рёбер; kinds задаёт metadata.kind вершинам."""
    kinds = kinds or {}
    return RoadGraph(
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
                edge_id=eid,
                start_id=a,
                end_id=b,
                polyline_xy=(nodes[a], nodes[b]),
            )
            for eid, a, b in edges
        },
    )


def _t_junction() -> RoadGraph:
    """Т-образный: 1-2-3 по прямой, отвод 2-4 вверх. Точка решения только 2."""
    return _graph(
        nodes={1: (0.0, 0.0), 2: (1.0, 0.0), 3: (2.0, 0.0), 4: (1.0, 1.0)},
        edges=[(10, 1, 2), (11, 2, 3), (12, 2, 4)],
    )


def test_degrees_ignore_edge_direction() -> None:
    graph = _t_junction()

    degrees = undirected_degrees(graph)

    assert degrees == {1: 1, 2: 3, 3: 1, 4: 1}


def test_decision_points_are_vertices_with_degree_not_two() -> None:
    topology = build_topology(_t_junction())

    # Ветвление и три тупика: у всех степень не равна двум.
    assert topology.decision_points == frozenset({1, 2, 3, 4})


def test_pass_through_vertex_is_not_a_decision_point() -> None:
    # Прямая 1-2-3: у вершины 2 степень два, решений она не принимает.
    graph = _graph(
        nodes={1: (0.0, 0.0), 2: (1.0, 0.0), 3: (2.0, 0.0)},
        edges=[(10, 1, 2), (11, 2, 3)],
    )

    topology = build_topology(graph)

    assert topology.decision_points == frozenset({1, 3})


def test_chain_spans_pass_through_vertices() -> None:
    graph = _graph(
        nodes={1: (0.0, 0.0), 2: (1.0, 0.0), 3: (2.0, 0.0)},
        edges=[(10, 1, 2), (11, 2, 3)],
    )

    topology = build_topology(graph)
    chains = topology.chains_between(1, 3)

    assert len(chains) == 1
    assert chains[0].vertices == (1, 2, 3)
    assert chains[0].interior == (2,)
    assert chains[0].polyline_xy == ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0))


def test_chain_orients_polyline_along_travel_even_against_stored_edge() -> None:
    # Ребро записано как 2 -> 1, а цепочка идёт 1 -> 2.
    graph = _graph(
        nodes={1: (0.0, 0.0), 2: (1.0, 0.0), 3: (2.0, 0.0)},
        edges=[(10, 2, 1), (11, 2, 3)],
    )

    topology = build_topology(graph)
    forward = topology.chains_between(1, 3)[0]

    # Полилиния идёт по ходу движения, а не как записано в графе.
    assert forward.polyline_xy == ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0))


def test_chains_exist_in_both_directions() -> None:
    topology = build_topology(_t_junction())

    assert topology.chains_between(2, 4)
    assert topology.chains_between(4, 2)


def test_neighbors_lists_reachable_decision_points() -> None:
    topology = build_topology(_t_junction())

    assert sorted(topology.neighbors(2)) == [1, 3, 4]
    assert topology.neighbors(4) == [2]


def test_kind_junction_forces_decision_point_at_degree_two() -> None:
    graph = _graph(
        nodes={1: (0.0, 0.0), 2: (1.0, 0.0), 3: (2.0, 0.0)},
        edges=[(10, 1, 2), (11, 2, 3)],
        kinds={2: KIND_DECISION},
    )

    topology = build_topology(graph)

    assert 2 in topology.decision_points
    # Цепочка теперь обрывается на вершине 2, а не проходит сквозь.
    assert topology.chains_between(1, 3) == []
    assert topology.chains_between(1, 2)


def test_kind_geometry_excludes_branching_vertex_from_decisions() -> None:
    # Вершина 2 ветвится, но объявлена геометрической: решений не принимает.
    graph = _t_junction()
    graph.nodes[2] = Node(
        node_id=2, x=1.0, y=0.0, metadata={KIND_KEY: KIND_PASSTHROUGH}
    )

    topology = build_topology(graph)

    assert 2 not in topology.decision_points
    # Через ветвящуюся вершину цепочку провести нельзя: путь обрывается.
    assert topology.chains == ()


def test_chain_reversed_flips_ends_and_polyline() -> None:
    graph = _graph(
        nodes={1: (0.0, 0.0), 2: (1.0, 0.0), 3: (2.0, 0.0)},
        edges=[(10, 1, 2), (11, 2, 3)],
    )

    chain = build_topology(graph).chains_between(1, 3)[0]
    back = chain.reversed()

    assert (back.start, back.end) == (3, 1)
    assert back.vertices == (3, 2, 1)
    assert back.polyline_xy == ((2.0, 0.0), (1.0, 0.0), (0.0, 0.0))


def test_parallel_chains_are_both_reported() -> None:
    # Два разных пути между точками решений 1 и 4: через 2 и через 3.
    # Отводы 5 и 6 нужны, чтобы 1 и 4 ветвились и стали точками решений.
    graph = _graph(
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

    topology = build_topology(graph)

    assert {1, 4} <= topology.decision_points
    # Неоднозначность видна сразу: это ловит проверка единственности цепочки.
    assert len(topology.chains_between(1, 4)) == 2


def test_pure_ring_has_no_decision_points() -> None:
    """Замкнутое кольцо без отводов не даёт ни точек решений, ни цепочек.

    У всех вершин степень два, ветвиться негде. Чтобы планировать по такому
    кольцу, хотя бы одну вершину надо объявить точкой решения через metadata.
    """
    ring = _graph(
        nodes={1: (0.0, 0.0), 2: (1.0, 1.0), 3: (2.0, 0.0), 4: (1.0, -1.0)},
        edges=[(10, 1, 2), (11, 2, 3), (12, 3, 4), (13, 4, 1)],
    )

    topology = build_topology(ring)

    assert topology.decision_points == frozenset()
    assert topology.chains == ()


def test_declared_vertex_makes_a_ring_planable() -> None:
    ring = _graph(
        nodes={1: (0.0, 0.0), 2: (1.0, 1.0), 3: (2.0, 0.0), 4: (1.0, -1.0)},
        edges=[(10, 1, 2), (11, 2, 3), (12, 3, 4), (13, 4, 1)],
        kinds={1: KIND_DECISION, 3: KIND_DECISION},
    )

    topology = build_topology(ring)

    assert topology.decision_points == frozenset({1, 3})
    # Две дуги кольца между объявленными вершинами.
    assert len(topology.chains_between(1, 3)) == 2


def test_empty_graph_has_no_chains() -> None:
    topology = build_topology(RoadGraph())

    assert topology.decision_points == frozenset()
    assert topology.chains == ()
