import json

import pytest

from rtk2026_city_nav.lane import LanePose
from rtk2026_city_nav.poses_io import (
    FORMAT_VERSION,
    LegPoses,
    PosesFile,
    from_dict,
    generate,
    graph_fingerprint,
    load,
    merge,
    save,
    to_dict,
)
from rtk2026_city_nav.topology import build_topology
from rtk2026_pose_graph.model import Node, OrientedEdge, RoadGraph

NODES: dict[int, tuple[float, float]] = {
    0: (0.0, 0.0),
    10: (1.0, 0.0),
    20: (0.0, 1.0),
    30: (0.0, -1.0),
}
EDGES: list[tuple[int, int, int]] = [(1, 0, 10), (2, 0, 20), (3, 0, 30)]


def _graph(
    nodes: dict[int, tuple[float, float]] | None = None,
) -> RoadGraph:
    nodes = nodes or NODES
    return RoadGraph(
        nodes={nid: Node(node_id=nid, x=xy[0], y=xy[1]) for nid, xy in nodes.items()},
        edges={
            eid: OrientedEdge(
                edge_id=eid, start_id=a, end_id=b, polyline_xy=(nodes[a], nodes[b])
            )
            for eid, a, b in EDGES
        },
    )


def _generate(graph: RoadGraph | None = None, **overrides: float) -> PosesFile:
    params: dict[str, float] = {"lane_offset_m": 0.2}
    params.update(overrides)
    return generate(build_topology(graph or _graph()), **params)  # type: ignore[arg-type]


def test_generate_covers_every_chain_in_both_directions() -> None:
    file = _generate()

    keys = {leg.key for leg in file.legs}
    # Три луча, каждый в обе стороны.
    assert keys == {(0, 10), (10, 0), (0, 20), (20, 0), (0, 30), (30, 0)}


def test_opposite_directions_get_different_poses() -> None:
    """У каждого направления своя полоса, значит и свои позы."""
    file = _generate()

    forward = file.leg(0, 10)
    backward = file.leg(10, 0)
    assert forward is not None and backward is not None

    # Едем на восток - полоса южнее, на запад - севернее.
    assert forward.poses[0].y == pytest.approx(-0.2)
    assert backward.poses[0].y == pytest.approx(+0.2)


def test_params_are_recorded() -> None:
    file = _generate(pose_step_m=0.5, miter_limit=3.0)

    assert file.params["lane_offset_m"] == pytest.approx(0.2)
    assert file.params["pose_step_m"] == pytest.approx(0.5)
    assert file.params["miter_limit"] == pytest.approx(3.0)


def test_fingerprint_is_stable_and_order_independent() -> None:
    first = graph_fingerprint(_graph())
    second = graph_fingerprint(_graph())

    assert first == second
    assert first.startswith("sha256:")


def test_fingerprint_changes_when_geometry_moves() -> None:
    moved = dict(NODES)
    moved[10] = (1.5, 0.0)

    assert graph_fingerprint(_graph()) != graph_fingerprint(_graph(moved))


def test_fingerprint_changes_when_a_vertex_role_changes() -> None:
    graph = _graph()
    plain = graph_fingerprint(graph)

    graph.nodes[0] = Node(node_id=0, x=0.0, y=0.0, metadata={"kind": "geometry"})

    assert graph_fingerprint(graph) != plain


def test_merge_regenerates_untouched_legs() -> None:
    existing = _generate()
    fresh = _generate()

    merged, report = merge(existing, fresh)

    assert len(report.regenerated) == len(fresh.legs)
    assert report.kept_manual == ()
    assert merged.legs == fresh.legs


def test_merge_keeps_manual_edits() -> None:
    existing = _generate()
    nudged = LegPoses(
        start=0,
        end=10,
        poses=(LanePose(x=0.5, y=-0.9, yaw=0.0),),
        manual=True,
    )
    existing = PosesFile(
        graph_fingerprint=existing.graph_fingerprint,
        params=existing.params,
        legs=tuple(nudged if leg.key == (0, 10) else leg for leg in existing.legs),
    )

    merged, report = merge(existing, _generate())

    assert (0, 10) in report.kept_manual
    assert (0, 10) not in report.regenerated

    kept = merged.leg(0, 10)
    assert kept is not None
    assert kept.manual is True
    # Ровно то, что было записано руками.
    assert kept.poses[0].y == pytest.approx(-0.9)


def test_manual_edit_under_a_different_graph_is_flagged_stale() -> None:
    """Правка могла относиться к прежней геометрии — это надо видеть."""
    old = _generate()
    old = PosesFile(
        graph_fingerprint=old.graph_fingerprint,
        params=old.params,
        legs=tuple(
            LegPoses(leg.start, leg.end, leg.poses, manual=leg.key == (0, 10))
            for leg in old.legs
        ),
    )

    moved = dict(NODES)
    moved[10] = (1.5, 0.0)
    merged, report = merge(old, _generate(_graph(moved)))

    assert (0, 10) in report.kept_manual
    assert (0, 10) in report.stale_manual
    assert "УСТАРЕВШИХ РУЧНЫХ" in report.summary()
    assert any("прежней геометрии" in line for line in report.details())
    # Правка всё равно сохранена: решать, что с ней делать, человеку.
    assert merged.leg(0, 10) is not None


def test_changed_parameters_also_make_manual_edits_stale() -> None:
    old = _generate(lane_offset_m=0.2)
    old = PosesFile(
        graph_fingerprint=old.graph_fingerprint,
        params=old.params,
        legs=tuple(
            LegPoses(leg.start, leg.end, leg.poses, manual=leg.key == (0, 10))
            for leg in old.legs
        ),
    )

    merged, report = merge(old, _generate(lane_offset_m=0.3))

    assert ("lane_offset_m", 0.2, 0.3) in report.changed_params
    assert (0, 10) in report.stale_manual
    assert any("lane_offset_m" in line for line in report.details())
    del merged


def test_merge_reports_legs_that_left_the_graph() -> None:
    existing = _generate()
    stripped = PosesFile(
        graph_fingerprint=existing.graph_fingerprint,
        params=existing.params,
        legs=existing.legs
        + (LegPoses(start=99, end=98, poses=(LanePose(0.0, 0.0, 0.0),)),),
    )

    merged, report = merge(stripped, _generate())

    assert (99, 98) in report.removed
    assert merged.leg(99, 98) is None
    assert any("больше нет" in line for line in report.details())


def test_merge_reports_new_legs() -> None:
    smaller = _generate()
    smaller = PosesFile(
        graph_fingerprint=smaller.graph_fingerprint,
        params=smaller.params,
        legs=tuple(leg for leg in smaller.legs if leg.key != (0, 30)),
    )

    _merged, report = merge(smaller, _generate())

    assert (0, 30) in report.added


def test_roundtrip_through_json_preserves_everything() -> None:
    original = _generate(pose_step_m=0.4)
    original = PosesFile(
        graph_fingerprint=original.graph_fingerprint,
        params=original.params,
        legs=tuple(
            LegPoses(leg.start, leg.end, leg.poses, manual=leg.key == (0, 20))
            for leg in original.legs
        ),
    )

    restored = from_dict(json.loads(json.dumps(to_dict(original))))

    assert restored.graph_fingerprint == original.graph_fingerprint
    assert restored.params == original.params
    assert restored.manual_keys == original.manual_keys
    assert len(restored.legs) == len(original.legs)


def test_saved_file_is_readable_and_diff_friendly(tmp_path) -> None:
    path = tmp_path / "poses.json"
    save(path, _generate())

    text = path.read_text(encoding="utf-8")

    # Отступы и перевод строки в конце: файл правится руками и коммитится.
    assert "\n  " in text
    assert text.endswith("\n")

    # Координаты округлены, а не с плавающим хвостом на 17 знаков.
    data = json.loads(text)
    for leg in data["legs"]:
        for pose in leg["poses"]:
            assert len(str(pose["x"]).split(".")[-1]) <= 4


def test_save_then_load_gives_the_same_file(tmp_path) -> None:
    path = tmp_path / "poses.json"
    original = _generate()

    save(path, original)
    restored = load(path)

    assert restored.graph_fingerprint == original.graph_fingerprint
    assert {leg.key for leg in restored.legs} == {leg.key for leg in original.legs}


def test_wrong_format_version_is_rejected() -> None:
    with pytest.raises(ValueError, match="версия формата"):
        from_dict({"version": FORMAT_VERSION + 1, "legs": []})


def test_missing_legs_list_is_rejected() -> None:
    with pytest.raises(ValueError, match="legs"):
        from_dict({"version": FORMAT_VERSION})


def test_leg_without_endpoints_is_rejected() -> None:
    with pytest.raises(ValueError, match="from"):
        from_dict({"version": FORMAT_VERSION, "legs": [{"poses": []}]})


def test_missing_leg_returns_none() -> None:
    file = _generate()

    assert file.leg(777, 888) is None
