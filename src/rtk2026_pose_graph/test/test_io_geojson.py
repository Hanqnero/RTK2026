import pytest

from rtk2026_pose_graph.io_geojson import load_geojson_dict


def _point(node_id: int, x: float, y: float) -> dict:
    return {
        "type": "Feature",
        "geometry": {"type": "Point", "coordinates": [x, y]},
        "properties": {"id": node_id, "frame": "map"},
    }


def _edge(
    edge_id: int,
    start_id: int,
    end_id: int,
    *,
    coordinates: list | None = None,
    cost: float = 0.0,
    overridable: bool = True,
    hard_side: str | None = None,
) -> dict:
    props: dict = {
        "id": edge_id,
        "startid": start_id,
        "endid": end_id,
        "cost": cost,
        "overridable": overridable,
    }
    if hard_side is not None:
        props["corridor_hard_side"] = hard_side
    geometry: dict = {"type": "LineString"}
    if coordinates is not None:
        geometry["coordinates"] = coordinates
    return {"type": "Feature", "geometry": geometry, "properties": props}


def test_load_nodes_and_edge_with_explicit_polyline() -> None:
    data = {
        "type": "FeatureCollection",
        "features": [
            _point(1, 0.0, 0.0),
            _point(2, 1.0, 0.0),
            _edge(100, 1, 2, coordinates=[[0.0, 0.0], [0.5, 0.1], [1.0, 0.0]]),
        ],
    }

    graph = load_geojson_dict(data)

    assert set(graph.nodes) == {1, 2}
    assert graph.nodes[1].x == 0.0
    assert graph.nodes[2].y == 0.0

    edge = graph.edges[100]
    assert (edge.start_id, edge.end_id) == (1, 2)
    assert edge.polyline_xy == ((0.0, 0.0), (0.5, 0.1), (1.0, 0.0))


def test_load_edge_without_geometry_falls_back_to_node_positions() -> None:
    # Формат допускает ребро без явной геометрии - тогда полилиния
    # строится из координат начального и конечного узла.
    data = {
        "type": "FeatureCollection",
        "features": [
            _point(1, 0.0, 0.0),
            _point(2, 3.0, 4.0),
            _edge(100, 1, 2),
        ],
    }

    graph = load_geojson_dict(data)

    assert graph.edges[100].polyline_xy == ((0.0, 0.0), (3.0, 4.0))


def test_load_edge_with_corridor_hard_side_and_cost() -> None:
    data = {
        "type": "FeatureCollection",
        "features": [
            _point(1, 0.0, 0.0),
            _point(2, 1.0, 0.0),
            _edge(
                100, 1, 2,
                coordinates=[[0.0, 0.0], [1.0, 0.0]],
                cost=2.5,
                overridable=False,
                hard_side="left",
            ),
        ],
    }

    edge = load_geojson_dict(data).edges[100]

    assert edge.cost == 2.5
    assert edge.overridable is False
    assert edge.corridor_hard_side == "left"


def test_load_ignores_edge_missing_required_properties() -> None:
    data = {
        "type": "FeatureCollection",
        "features": [
            _point(1, 0.0, 0.0),
            _point(2, 1.0, 0.0),
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [1.0, 0.0]]},
                "properties": {"startid": 1, "endid": 2},  # нет "id"
            },
        ],
    }

    graph = load_geojson_dict(data)

    assert graph.edges == {}


def test_load_rejects_duplicate_node_id() -> None:
    data = {
        "type": "FeatureCollection",
        "features": [_point(1, 0.0, 0.0), _point(1, 5.0, 5.0)],
    }

    with pytest.raises(ValueError, match="дубликат"):
        load_geojson_dict(data)


def test_load_rejects_edge_without_geometry_and_unknown_nodes() -> None:
    data = {
        "type": "FeatureCollection",
        "features": [_edge(100, 1, 2)],  # ни узлов, ни геометрии
    }

    with pytest.raises(ValueError):
        load_geojson_dict(data)


def test_load_rejects_non_feature_collection() -> None:
    with pytest.raises(ValueError, match="FeatureCollection"):
        load_geojson_dict({"type": "Feature"})
