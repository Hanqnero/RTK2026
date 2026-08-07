from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph


def _build_graph() -> RoadGraph:
    nodes = {
        1: Node(node_id=1, x=0.0, y=0.0),
        2: Node(node_id=2, x=1.0, y=0.0),
        3: Node(node_id=3, x=1.0, y=1.0),
    }
    edges = {
        100: OrientedEdge(edge_id=100, start_id=1, end_id=2),
        101: OrientedEdge(edge_id=101, start_id=1, end_id=3),
        102: OrientedEdge(edge_id=102, start_id=2, end_id=3),
    }
    return RoadGraph(nodes=nodes, edges=edges)


def test_outgoing_and_incoming_edge_ids() -> None:
    graph = _build_graph()

    assert sorted(graph.outgoing_edge_ids(1)) == [100, 101]
    assert graph.outgoing_edge_ids(3) == []
    assert sorted(graph.incoming_edge_ids(3)) == [101, 102]
    assert graph.incoming_edge_ids(1) == []


def test_neighbor_ids_dedup_and_keep_first_order() -> None:
    graph = _build_graph()
    graph.edges[103] = OrientedEdge(edge_id=103, start_id=1, end_id=2)
    graph.reindex()

    # Узел 2 достижим двумя параллельными рёбрами (100 и 103), но в списке
    # соседей должен появиться один раз.
    assert graph.outgoing_neighbor_ids(1) == [2, 3]
    assert graph.incoming_neighbor_ids(3) == [1, 2]


def test_edges_between_returns_all_parallel_edges() -> None:
    graph = _build_graph()
    graph.edges[103] = OrientedEdge(edge_id=103, start_id=1, end_id=2)
    graph.reindex()

    found = graph.edges_between(1, 2)

    assert sorted(edge.edge_id for edge in found) == [100, 103]


def test_edge_toward_neighbor_and_has_edge() -> None:
    graph = _build_graph()

    edge = graph.edge_toward_neighbor(1, 2)
    assert edge is not None
    assert edge.edge_id == 100

    assert graph.edge_toward_neighbor(3, 1) is None
    assert graph.has_edge(1, 2) is True
    assert graph.has_edge(3, 1) is False


def test_reindex_picks_up_manual_edits() -> None:
    graph = _build_graph()

    graph.edges[200] = OrientedEdge(edge_id=200, start_id=3, end_id=1)
    # До reindex индексы ещё старые.
    assert graph.outgoing_edge_ids(3) == []

    graph.reindex()
    assert graph.outgoing_edge_ids(3) == [200]


def test_metadata_carries_arbitrary_keys() -> None:
    edge = OrientedEdge(
        edge_id=1,
        start_id=1,
        end_id=2,
        metadata={"corridor_hard_side": "left", "speed_limit": 0.4},
    )

    assert edge.meta("corridor_hard_side") == "left"
    assert edge.meta("speed_limit") == 0.4
    # Неизвестный ключ отдаёт default, а не падает.
    assert edge.meta("is_crosswalk") is None
    assert edge.meta("is_crosswalk", False) is False


def test_node_metadata_and_xy() -> None:
    node = Node(node_id=7, x=1.5, y=-2.0, metadata={"kind": "junction"})

    assert node.xy == (1.5, -2.0)
    assert node.meta("kind") == "junction"


def test_reversed_edge_flips_direction_and_polyline() -> None:
    edge = OrientedEdge(
        edge_id=100,
        start_id=1,
        end_id=2,
        polyline_xy=((0.0, 0.0), (0.5, 0.2), (1.0, 0.0)),
        cost=3.0,
        overridable=False,
        metadata={"corridor_hard_side": "left"},
    )

    back = edge.reversed()

    assert (back.start_id, back.end_id) == (2, 1)
    assert back.polyline_xy == ((1.0, 0.0), (0.5, 0.2), (0.0, 0.0))
    assert back.cost == 3.0
    assert back.overridable is False
    # Значение аннотации не переворачивается вместе с направлением.
    assert back.meta("corridor_hard_side") == "left"
    # Копия, а не та же ссылка.
    assert back.metadata is not edge.metadata


def test_reversed_edge_can_take_new_id() -> None:
    edge = OrientedEdge(edge_id=100, start_id=1, end_id=2)

    assert edge.reversed().edge_id == 100
    assert edge.reversed(edge_id=555).edge_id == 555


def test_metadata_excluded_from_equality_keeps_elements_hashable() -> None:
    plain = OrientedEdge(edge_id=1, start_id=1, end_id=2)
    annotated = OrientedEdge(edge_id=1, start_id=1, end_id=2, metadata={"a": 1})

    # Тождество задаётся структурой, а не аннотациями.
    assert plain == annotated
    assert len({plain, annotated}) == 1
    assert len({Node(node_id=1, x=0.0, y=0.0, metadata={"a": 1})}) == 1


def test_node_and_edge_are_frozen() -> None:
    node = Node(node_id=1, x=0.0, y=0.0)
    edge = OrientedEdge(edge_id=1, start_id=1, end_id=2)

    for target, attribute, value in ((node, "x", 5.0), (edge, "cost", 5.0)):
        try:
            setattr(target, attribute, value)
        except AttributeError:
            continue
        raise AssertionError(f"{type(target).__name__} обязан быть неизменяемым")


def test_validate_accepts_consistent_graph() -> None:
    nodes = {
        1: Node(node_id=1, x=0.0, y=0.0),
        2: Node(node_id=2, x=1.0, y=0.0),
    }
    edges = {
        100: OrientedEdge(
            edge_id=100, start_id=1, end_id=2, polyline_xy=((0.0, 0.0), (1.0, 0.0))
        )
    }

    assert RoadGraph(nodes=nodes, edges=edges).validate() == []


def test_validate_reports_dangling_references_and_bad_polyline() -> None:
    nodes = {1: Node(node_id=1, x=0.0, y=0.0)}
    edges = {
        100: OrientedEdge(edge_id=100, start_id=1, end_id=99, polyline_xy=((0.0, 0.0),)),
    }

    problems = RoadGraph(nodes=nodes, edges=edges).validate()

    assert any("end_id=99" in problem for problem in problems)
    assert any("полилиния" in problem for problem in problems)


def test_validate_reports_key_mismatch() -> None:
    graph = RoadGraph(
        nodes={5: Node(node_id=7, x=0.0, y=0.0)},
        edges={},
    )

    assert any("node_id=7" in problem for problem in graph.validate())
