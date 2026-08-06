from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

# Сторона «жёсткой» границы коридора относительно направления движения по ребру (start → end).
CorridorHardSide = Literal["left", "right"]


@dataclass(frozen=True)
class Node:
    """Вершина графа в плоскости карты (обычно frame=map)."""

    node_id: int
    x: float
    y: float
    frame: str = "map"


@dataclass(frozen=True)
class OrientedEdge:
    """Ориентированное ребро: движение от start_id к end_id.

    polyline_xy — последовательность точек вдоль полосы (минимум две, если заданы явно).
    Вариант B: corridor_hard_side задаёт, с какой стороны от направления движения
    находится непересекаемая граница (вторая сторона — зона costmap / препятствий).
    """

    edge_id: int
    start_id: int
    end_id: int
    polyline_xy: tuple[tuple[float, float], ...] = ()
    cost: float = 0.0
    overridable: bool = True
    corridor_hard_side: CorridorHardSide | None = None


@dataclass
class RoadGraph:
    """Весь граф: узлы и рёбра по идентификаторам."""

    nodes: dict[int, Node] = field(default_factory=dict)
    edges: dict[int, OrientedEdge] = field(default_factory=dict)

    def outgoing_edge_ids(self, node_id: int) -> list[int]:
        """Исходящие рёбра из узла (по start_id)."""
        return [eid for eid, e in self.edges.items() if e.start_id == node_id]

    def outgoing_neighbor_ids(self, node_id: int) -> list[int]:
        """Соседние узлы по исходящим рёбрам (end_id), без дубликатов, порядок по первому ребру."""
        seen: list[int] = []
        for eid in self.outgoing_edge_ids(node_id):
            end = self.edges[eid].end_id
            if end not in seen:
                seen.append(end)
        return seen

    def edge_toward_neighbor(self, start_id: int, neighbor_id: int) -> OrientedEdge | None:
        """Исходящее ребро start_id → neighbor_id (первое по порядку, если параллельных несколько)."""
        for eid in self.outgoing_edge_ids(start_id):
            e = self.edges[eid]
            if e.end_id == neighbor_id:
                return e
        return None
