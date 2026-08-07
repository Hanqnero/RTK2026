"""Модель дорожного графа на карте: вершины, ориентированные рёбра, запросы к ним.

Прикладные свойства вершин и рёбер хранятся в ``metadata`` — свободном
наборе ключей, который граф передаёт как есть, а трактует тот код, которому
они нужны. Так же устроен ``nav2_route``.

``Node`` и ``OrientedEdge`` неизменяемы. ``RoadGraph`` строит индексы
смежности при создании и рассчитан на схему «загрузили — много раз
спросили»: после правки ``nodes``/``edges`` руками нужен
:meth:`RoadGraph.reindex`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class Node:
    """Вершина графа в плоскости карты."""

    node_id: int
    x: float
    y: float
    frame: str = "map"

    #: Произвольные аннотации вершины. Не участвуют в сравнении, поэтому
    #: ``Node`` остаётся хешируемым.
    metadata: dict[str, Any] = field(default_factory=dict[str, Any], compare=False)

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)

    def meta(self, key: str, default: Any = None) -> Any:
        """Значение аннотации или ``default``, если ключа нет."""
        return self.metadata.get(key, default)


@dataclass(frozen=True)
class OrientedEdge:
    """Ориентированное ребро: движение от ``start_id`` к ``end_id``.

    ``polyline_xy`` — путь вдоль ребра, минимум две точки. Если в источнике
    геометрии нет, загрузчик подставляет прямую между вершинами.
    """

    edge_id: int
    start_id: int
    end_id: int
    polyline_xy: tuple[tuple[float, float], ...] = ()
    cost: float = 0.0
    overridable: bool = True

    #: Произвольные аннотации ребра: ограничение скорости, тип полосы,
    #: что угодно ещё. Не участвуют в сравнении, как и у :class:`Node`.
    metadata: dict[str, Any] = field(default_factory=dict[str, Any], compare=False)

    def meta(self, key: str, default: Any = None) -> Any:
        """Значение аннотации или ``default``, если ключа нет."""
        return self.metadata.get(key, default)

    def reversed(self, *, edge_id: int | None = None) -> OrientedEdge:
        """Ребро в обратную сторону с развёрнутой полилинией.

        Аннотации копируются без изменений. Если аннотация зависит от
        направления движения, разворачивать её значение должен тот, кто
        её проставил.
        """
        return OrientedEdge(
            edge_id=self.edge_id if edge_id is None else edge_id,
            start_id=self.end_id,
            end_id=self.start_id,
            polyline_xy=tuple(reversed(self.polyline_xy)),
            cost=self.cost,
            overridable=self.overridable,
            metadata=dict(self.metadata),
        )


@dataclass
class RoadGraph:
    """Граф целиком: вершины и рёбра по идентификаторам, плюс индексы смежности."""

    nodes: dict[int, Node] = field(default_factory=dict[int, Node])
    edges: dict[int, OrientedEdge] = field(default_factory=dict[int, OrientedEdge])

    # Индексы смежности: без них запрос соседей — полный проход по всем
    # рёбрам, а его делают на каждом тике управления.
    _outgoing: dict[int, list[int]] = field(
        default_factory=dict[int, list[int]], init=False, repr=False, compare=False
    )
    _incoming: dict[int, list[int]] = field(
        default_factory=dict[int, list[int]], init=False, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        self.reindex()

    def reindex(self) -> None:
        """Перестроить индексы смежности после ручной правки словарей."""
        outgoing: dict[int, list[int]] = {}
        incoming: dict[int, list[int]] = {}

        for edge_id, edge in self.edges.items():
            outgoing.setdefault(edge.start_id, []).append(edge_id)
            incoming.setdefault(edge.end_id, []).append(edge_id)

        self._outgoing = outgoing
        self._incoming = incoming

    def outgoing_edge_ids(self, node_id: int) -> list[int]:
        """Идентификаторы рёбер, выходящих из вершины."""
        return list(self._outgoing.get(node_id, ()))

    def incoming_edge_ids(self, node_id: int) -> list[int]:
        """Идентификаторы рёбер, входящих в вершину."""
        return list(self._incoming.get(node_id, ()))

    def outgoing_neighbor_ids(self, node_id: int) -> list[int]:
        """Вершины, достижимые по исходящим рёбрам, без повторов."""
        return _unique([self.edges[eid].end_id for eid in self._outgoing.get(node_id, ())])

    def incoming_neighbor_ids(self, node_id: int) -> list[int]:
        """Вершины, из которых есть ребро в эту, без повторов."""
        return _unique([self.edges[eid].start_id for eid in self._incoming.get(node_id, ())])

    def edges_between(self, start_id: int, end_id: int) -> list[OrientedEdge]:
        """Все рёбра ``start_id -> end_id``: их может быть несколько (параллельные)."""
        return [
            self.edges[eid]
            for eid in self._outgoing.get(start_id, ())
            if self.edges[eid].end_id == end_id
        ]

    def edge_toward_neighbor(self, start_id: int, neighbor_id: int) -> OrientedEdge | None:
        """Первое ребро ``start_id -> neighbor_id`` или ``None``.

        Когда параллельных рёбер несколько, выбор «первого» произволен —
        используйте :meth:`edges_between`, если разница важна.
        """
        found = self.edges_between(start_id, neighbor_id)
        return found[0] if found else None

    def has_edge(self, start_id: int, end_id: int) -> bool:
        """Есть ли хотя бы одно ребро в этом направлении."""
        return bool(self.edges_between(start_id, end_id))

    def validate(self) -> list[str]:
        """Найти ссылочные и геометрические дефекты графа.

        Возвращает список описаний проблем; пустой список — граф целостен.
        Исключение не бросает: вызывающий сам решает, писать в лог или падать.
        """
        problems: list[str] = []

        for node_id, node in self.nodes.items():
            if node_id != node.node_id:
                problems.append(
                    f"вершина под ключом {node_id} имеет node_id={node.node_id}"
                )

        for edge_id, edge in self.edges.items():
            if edge_id != edge.edge_id:
                problems.append(f"ребро под ключом {edge_id} имеет edge_id={edge.edge_id}")
            if edge.start_id not in self.nodes:
                problems.append(f"ребро {edge_id}: нет вершины start_id={edge.start_id}")
            if edge.end_id not in self.nodes:
                problems.append(f"ребро {edge_id}: нет вершины end_id={edge.end_id}")
            if len(edge.polyline_xy) < 2:
                problems.append(f"ребро {edge_id}: полилиния короче двух точек")

        return problems


def _unique(values: list[int]) -> list[int]:
    """Убрать повторы, сохранив порядок первого появления."""
    seen: dict[int, None] = {}
    for value in values:
        seen.setdefault(value, None)
    return list(seen)
