"""Позы участков в отдельном файле, который можно править руками.

Позы вычисляются из графа, но на практике часть точек приходится отодвигать:
где-то мешает препятствие, где-то расчётная траектория задевает бордюр.
Поэтому они не строятся на каждом запуске заново, а лежат в файле, который
правится и коммитится.

Из этого следуют два требования, и оба они про то, чтобы правки не пропали
молча.

Повторная генерация сохраняет записи, помеченные ``manual``, а в отчёте
пишет, сколько их и какие. Иначе после любой правки графа ручная работа
исчезала бы без следа.

В файле хранится отпечаток графа, из которого он сгенерирован. Если граф
изменился, ручные правки могли относиться к прежней геометрии, и это надо
видеть, а не выяснять на трассе.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rtk2026_city_nav.lane import RIGHT_HAND_TRAFFIC, LanePose, resample_along_chain
from rtk2026_city_nav.topology import Topology
from rtk2026_pose_graph.model import RoadGraph

#: Версия формата файла. Растёт, когда меняется разметка, а не содержимое.
FORMAT_VERSION = 1

#: Точность записи координат. Десятая доля миллиметра: точнее не нужно,
#: а более длинные числа мешают читать и править файл руками.
_XY_DIGITS = 4
_YAW_DIGITS = 5


@dataclass(frozen=True)
class LegPoses:
    """Позы одного участка между точками решений."""

    start: int
    end: int
    poses: tuple[LanePose, ...]
    #: Правлено руками. Повторная генерация такие записи не трогает.
    manual: bool = False

    @property
    def key(self) -> tuple[int, int]:
        return (self.start, self.end)


@dataclass(frozen=True)
class PosesFile:
    """Содержимое файла поз."""

    #: Отпечаток графа, из которого сгенерировано.
    graph_fingerprint: str
    #: Параметры, с которыми считались позы. Хранятся отдельно от отпечатка,
    #: чтобы расхождение было видно по имени параметра, а не одним хешем.
    params: dict[str, float]
    legs: tuple[LegPoses, ...]
    version: int = FORMAT_VERSION

    def leg(self, start: int, end: int) -> LegPoses | None:
        """Позы участка или ``None``, если его в файле нет."""
        for item in self.legs:
            if item.key == (start, end):
                return item
        return None

    @property
    def manual_keys(self) -> tuple[tuple[int, int], ...]:
        return tuple(item.key for item in self.legs if item.manual)


@dataclass
class MergeReport:
    """Что произошло при слиянии свежих поз с уже лежащими в файле."""

    #: Записи, оставленные как есть из-за пометки ``manual``.
    kept_manual: tuple[tuple[int, int], ...] = ()
    #: Записи, пересчитанные заново.
    regenerated: tuple[tuple[int, int], ...] = ()
    #: Участки, которых в файле не было.
    added: tuple[tuple[int, int], ...] = ()
    #: Записи, которым в графе больше нет соответствия.
    removed: tuple[tuple[int, int], ...] = ()
    #: Ручные правки, сделанные под другой граф либо другие параметры.
    stale_manual: tuple[tuple[int, int], ...] = ()
    #: Параметры, изменившиеся с прошлой генерации: имя, было, стало.
    changed_params: tuple[tuple[str, float, float], ...] = ()

    def summary(self) -> str:
        parts = [
            f"пересчитано {len(self.regenerated)}",
            f"сохранено ручных {len(self.kept_manual)}",
        ]
        if self.added:
            parts.append(f"добавлено {len(self.added)}")
        if self.removed:
            parts.append(f"удалено {len(self.removed)}")
        if self.stale_manual:
            parts.append(f"УСТАРЕВШИХ РУЧНЫХ {len(self.stale_manual)}")
        return ", ".join(parts)

    def details(self) -> tuple[str, ...]:
        """Построчный разбор для лога. Пусто, если ничего примечательного."""
        lines: list[str] = []

        for name, was, now in self.changed_params:
            lines.append(f"параметр {name}: было {was}, стало {now}")

        for start, end in self.stale_manual:
            lines.append(
                f"участок {start} -> {end}: ручная правка относится "
                "к прежней геометрии, проверить"
            )
        for start, end in self.kept_manual:
            lines.append(f"участок {start} -> {end}: оставлен ручной")
        for start, end in self.removed:
            lines.append(
                f"участок {start} -> {end}: в графе больше нет, запись убрана"
            )
        return tuple(lines)


def graph_fingerprint(graph: RoadGraph) -> str:
    """Отпечаток графа: вершины, рёбра, геометрия и роли вершин.

    Меняется при любой правке, влияющей на позы. Не меняется от порядка
    записей в файле: всё сортируется перед хешированием.
    """
    parts: list[str] = []

    for node_id in sorted(graph.nodes):
        node = graph.nodes[node_id]
        kind = node.meta("kind") or ""
        parts.append(f"n{node_id}:{node.x:.6f},{node.y:.6f}:{kind}")

    for edge_id in sorted(graph.edges):
        edge = graph.edges[edge_id]
        points = ";".join(f"{x:.6f},{y:.6f}" for x, y in edge.polyline_xy)
        parts.append(f"e{edge_id}:{edge.start_id}>{edge.end_id}:{points}")

    digest = hashlib.sha256("|".join(parts).encode("utf-8")).hexdigest()
    return f"sha256:{digest[:16]}"


def generate(
    topology: Topology,
    *,
    lane_offset_m: float,
    pose_step_m: float = 0.0,
    traffic_side: int = RIGHT_HAND_TRAFFIC,
    miter_limit: float = 2.0,
) -> PosesFile:
    """Построить позы для всех участков графа.

    Участки берутся в обе стороны: у каждого направления своя полоса,
    поэтому и свои позы.
    """
    legs = [
        LegPoses(
            start=chain.start,
            end=chain.end,
            poses=resample_along_chain(
                chain.polyline_xy,
                lane_offset_m=lane_offset_m,
                step_m=pose_step_m,
                traffic_side=traffic_side,
                miter_limit=miter_limit,
            ),
        )
        for chain in topology.chains
    ]

    return PosesFile(
        graph_fingerprint=graph_fingerprint(topology.graph),
        params={
            "lane_offset_m": float(lane_offset_m),
            "pose_step_m": float(pose_step_m),
            "traffic_side": float(traffic_side),
            "miter_limit": float(miter_limit),
        },
        legs=tuple(sorted(legs, key=lambda item: item.key)),
    )


def merge(existing: PosesFile, fresh: PosesFile) -> tuple[PosesFile, MergeReport]:
    """Слить свежие позы с уже лежащими, сохранив ручные правки.

    Ручная запись переносится как есть. Если граф или параметры изменились,
    она попадает ещё и в ``stale_manual``: правка могла делаться под прежнюю
    геометрию, и молча оставлять её нельзя.
    """
    previous = {item.key: item for item in existing.legs}
    fingerprint_changed = existing.graph_fingerprint != fresh.graph_fingerprint

    changed_params = tuple(
        (name, existing.params[name], value)
        for name, value in sorted(fresh.params.items())
        if name in existing.params and existing.params[name] != value
    )
    settings_changed = fingerprint_changed or bool(changed_params)

    merged: list[LegPoses] = []
    kept: list[tuple[int, int]] = []
    regenerated: list[tuple[int, int]] = []
    added: list[tuple[int, int]] = []
    stale: list[tuple[int, int]] = []

    for item in fresh.legs:
        old = previous.pop(item.key, None)

        if old is None:
            merged.append(item)
            added.append(item.key)
            continue

        if old.manual:
            merged.append(old)
            kept.append(item.key)
            if settings_changed:
                stale.append(item.key)
            continue

        merged.append(item)
        regenerated.append(item.key)

    # Что осталось в previous - участков с такими концами в графе больше нет.
    removed = tuple(sorted(previous))

    report = MergeReport(
        kept_manual=tuple(kept),
        regenerated=tuple(regenerated),
        added=tuple(added),
        removed=removed,
        stale_manual=tuple(stale),
        changed_params=changed_params,
    )

    return (
        PosesFile(
            graph_fingerprint=fresh.graph_fingerprint,
            params=dict(fresh.params),
            legs=tuple(sorted(merged, key=lambda item: item.key)),
            version=fresh.version,
        ),
        report,
    )


def to_dict(file: PosesFile) -> dict[str, Any]:
    """Представление для записи в JSON."""
    return {
        "version": file.version,
        "graph_fingerprint": file.graph_fingerprint,
        "params": dict(sorted(file.params.items())),
        "legs": [
            {
                "from": item.start,
                "to": item.end,
                "manual": item.manual,
                "poses": [
                    {
                        "x": round(pose.x, _XY_DIGITS),
                        "y": round(pose.y, _XY_DIGITS),
                        "yaw": round(pose.yaw, _YAW_DIGITS),
                    }
                    for pose in item.poses
                ],
            }
            for item in file.legs
        ],
    }


def from_dict(data: dict[str, Any]) -> PosesFile:
    """Разобрать содержимое файла.

    :raises ValueError: версия формата не та либо структура не та.
    """
    version = int(data.get("version", 0))
    if version != FORMAT_VERSION:
        raise ValueError(
            f"версия формата {version}, ожидается {FORMAT_VERSION}"
        )

    raw_legs = data.get("legs")
    if not isinstance(raw_legs, list):
        raise ValueError("legs должен быть списком")

    legs: list[LegPoses] = []
    for entry in raw_legs:
        if not isinstance(entry, dict):
            continue
        if "from" not in entry or "to" not in entry:
            raise ValueError("у участка нет from либо to")

        legs.append(
            LegPoses(
                start=int(entry["from"]),
                end=int(entry["to"]),
                poses=tuple(
                    LanePose(
                        x=float(pose["x"]),
                        y=float(pose["y"]),
                        yaw=float(pose.get("yaw", 0.0)),
                    )
                    for pose in entry.get("poses", [])
                    if isinstance(pose, dict) and "x" in pose and "y" in pose
                ),
                manual=bool(entry.get("manual", False)),
            )
        )

    raw_params = data.get("params")
    params = (
        {str(k): float(v) for k, v in raw_params.items()}
        if isinstance(raw_params, dict)
        else {}
    )

    return PosesFile(
        graph_fingerprint=str(data.get("graph_fingerprint", "")),
        params=params,
        legs=tuple(sorted(legs, key=lambda item: item.key)),
        version=version,
    )


def load(path: str | Path) -> PosesFile:
    """Прочитать файл поз."""
    with Path(path).open(encoding="utf-8") as stream:
        return from_dict(json.load(stream))


def save(path: str | Path, file: PosesFile) -> None:
    """Записать файл поз."""
    with Path(path).open("w", encoding="utf-8") as stream:
        json.dump(to_dict(file), stream, ensure_ascii=False, indent=2)
        stream.write("\n")
