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


def test_outgoing_edge_ids_filters_by_start() -> None:
    graph = _build_graph()

    assert sorted(graph.outgoing_edge_ids(1)) == [100, 101]
    assert graph.outgoing_edge_ids(3) == []


def test_outgoing_neighbor_ids_dedups_and_keeps_first_order() -> None:
    graph = _build_graph()
    graph.edges[103] = OrientedEdge(edge_id=103, start_id=1, end_id=2)

    # Узел 2 достижим двумя параллельными рёбрами (100 и 103), но должен
    # появиться в списке соседей один раз.
    assert graph.outgoing_neighbor_ids(1) == [2, 3]


def test_edge_toward_neighbor_returns_first_match() -> None:
    graph = _build_graph()

    edge = graph.edge_toward_neighbor(1, 2)
    assert edge is not None
    assert edge.edge_id == 100


def test_edge_toward_neighbor_returns_none_when_absent() -> None:
    graph = _build_graph()

    assert graph.edge_toward_neighbor(3, 1) is None


def test_node_and_edge_are_frozen() -> None:
    node = Node(node_id=1, x=0.0, y=0.0)
    edge = OrientedEdge(edge_id=1, start_id=1, end_id=2)

    try:
        node.x = 5.0  # type: ignore[misc]
    except AttributeError:
        pass
    else:
        raise AssertionError("Node обязан быть неизменяемым")

    try:
        edge.cost = 5.0  # type: ignore[misc]
    except AttributeError:
        pass
    else:
        raise AssertionError("OrientedEdge обязан быть неизменяемым")
