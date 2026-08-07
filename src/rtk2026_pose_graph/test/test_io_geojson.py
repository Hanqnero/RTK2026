import pytest

from rtk2026_pose_graph.io_geojson import load_geojson_dict


def _point(node_id: int, x: float, y: float, **extra: object) -> dict:
    return {
        "type": "Feature",
        "geometry": {"type": "Point", "coordinates": [x, y]},
        "properties": {"id": node_id, "frame": "map", **extra},
    }


def _edge(
    edge_id: int,
    start_id: int,
    end_id: int,
    *,
    coordinates: list | None = None,
    geometry_type: str = "LineString",
    **extra: object,
) -> dict:
    geometry: dict = {"type": geometry_type}
    if coordinates is not None:
        geometry["coordinates"] = coordinates
    return {
        "type": "Feature",
        "geometry": geometry,
        "properties": {"id": edge_id, "startid": start_id, "endid": end_id, **extra},
    }


def _collection(*features: dict) -> dict:
    return {"type": "FeatureCollection", "features": list(features)}


def test_load_nodes_and_edge_with_explicit_polyline() -> None:
    data = _collection(
        _point(1, 0.0, 0.0),
        _point(2, 1.0, 0.0),
        _edge(100, 1, 2, coordinates=[[0.0, 0.0], [0.5, 0.1], [1.0, 0.0]]),
    )

    graph = load_geojson_dict(data)

    assert set(graph.nodes) == {1, 2}
    assert graph.nodes[1].xy == (0.0, 0.0)
    assert graph.nodes[2].frame == "map"

    edge = graph.edges[100]
    assert (edge.start_id, edge.end_id) == (1, 2)
    assert edge.polyline_xy == ((0.0, 0.0), (0.5, 0.1), (1.0, 0.0))
    assert graph.validate() == []


def test_load_edge_without_geometry_falls_back_to_node_positions() -> None:
    data = _collection(_point(1, 0.0, 0.0), _point(2, 3.0, 4.0), _edge(100, 1, 2))

    assert load_geojson_dict(data).edges[100].polyline_xy == ((0.0, 0.0), (3.0, 4.0))


def test_load_multilinestring_takes_first_line() -> None:
    data = _collection(
        _point(1, 0.0, 0.0),
        _point(2, 1.0, 0.0),
        _edge(
            100, 1, 2,
            geometry_type="MultiLineString",
            coordinates=[[[0.0, 0.0], [1.0, 0.0]], [[9.0, 9.0], [8.0, 8.0]]],
        ),
    )

    assert load_geojson_dict(data).edges[100].polyline_xy == ((0.0, 0.0), (1.0, 0.0))


def test_structural_properties_are_parsed_not_duplicated_in_metadata() -> None:
    data = _collection(
        _point(1, 0.0, 0.0),
        _point(2, 1.0, 0.0),
        _edge(100, 1, 2, coordinates=[[0.0, 0.0], [1.0, 0.0]], cost=2.5, overridable=False),
    )

    edge = load_geojson_dict(data).edges[100]

    assert edge.cost == 2.5
    assert edge.overridable is False
    # То, что загрузчик разобрал сам, в metadata не дублируется.
    assert edge.metadata == {}


def test_unknown_properties_flow_into_metadata_untouched() -> None:
    """Ключ прикладного правила проходит насквозь без правки загрузчика."""

    data = _collection(
        _point(1, 0.0, 0.0, kind="junction", note="перекрёсток у входа"),
        _point(2, 1.0, 0.0),
        _edge(
            100, 1, 2,
            coordinates=[[0.0, 0.0], [1.0, 0.0]],
            corridor_hard_side="left",
            speed_limit=0.35,
            operations=["AdjustSpeedLimit"],
        ),
    )

    graph = load_geojson_dict(data)

    assert graph.nodes[1].metadata == {"kind": "junction", "note": "перекрёсток у входа"}
    assert graph.nodes[2].metadata == {}

    edge = graph.edges[100]
    assert edge.meta("corridor_hard_side") == "left"
    assert edge.meta("speed_limit") == 0.35
    # Структуры, а не только скаляры: загрузчик ничего не приводит к типам.
    assert edge.meta("operations") == ["AdjustSpeedLimit"]


def test_load_ignores_features_missing_required_properties() -> None:
    data = _collection(
        _point(1, 0.0, 0.0),
        _point(2, 1.0, 0.0),
        {
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [1.0, 0.0]]},
            "properties": {"startid": 1, "endid": 2},  # нет "id"
        },
        # Вершина без id тоже пропускается.
        {
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [5.0, 5.0]},
            "properties": {"frame": "map"},
        },
    )

    graph = load_geojson_dict(data)

    assert graph.edges == {}
    assert set(graph.nodes) == {1, 2}


def test_load_skips_non_feature_entries() -> None:
    data = _collection(
        _point(1, 0.0, 0.0),
        {"type": "NotAFeature", "properties": {"id": 999}},
    )

    assert set(load_geojson_dict(data).nodes) == {1}


def test_duplicate_edge_id_keeps_first() -> None:
    data = _collection(
        _point(1, 0.0, 0.0),
        _point(2, 1.0, 0.0),
        _edge(100, 1, 2, coordinates=[[0.0, 0.0], [1.0, 0.0]]),
        _edge(100, 2, 1, coordinates=[[1.0, 0.0], [0.0, 0.0]]),
    )

    edge = load_geojson_dict(data).edges[100]

    assert (edge.start_id, edge.end_id) == (1, 2)


def test_load_rejects_duplicate_node_id() -> None:
    with pytest.raises(ValueError, match="дубликат"):
        load_geojson_dict(_collection(_point(1, 0.0, 0.0), _point(1, 5.0, 5.0)))


def test_load_rejects_edge_without_geometry_and_unknown_nodes() -> None:
    with pytest.raises(ValueError, match="не найдены узлы"):
        load_geojson_dict(_collection(_edge(100, 1, 2)))


def test_load_rejects_edge_with_single_point_geometry() -> None:
    data = _collection(
        _point(1, 0.0, 0.0),
        _point(2, 1.0, 0.0),
        _edge(100, 1, 2, coordinates=[[0.0, 0.0]]),
    )

    with pytest.raises(ValueError, match="минимум двух точек"):
        load_geojson_dict(data)


def test_load_rejects_non_feature_collection() -> None:
    with pytest.raises(ValueError, match="FeatureCollection"):
        load_geojson_dict({"type": "Feature"})

    with pytest.raises(ValueError, match="features"):
        load_geojson_dict({"type": "FeatureCollection", "features": "не список"})
