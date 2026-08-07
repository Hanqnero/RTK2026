"""Модель дорожного графа на карте: вершины, ориентированные рёбра, запросы к ним.

Ничего не знает ни про ROS, ни про робота, ни про конкретные правила движения.
Прикладной смысл живёт не в типах, а в ``metadata`` — свободном наборе
ключей у каждой вершины и ребра.

Почему metadata, а не типизированные поля
------------------------------------------

Соблазн описать нужный сценарий полем в структуре (``corridor_hard_side``,
``speed_limit``, ``is_crosswalk``) заканчивается тем, что общий модуль знает
про все кейсы сразу и меняется при появлении каждого нового. Здесь та же
схема, что у ``nav2_route``: граф хранит и отдаёт произвольные аннотации,
а трактует их тот код, которому они нужны. Добавление правила движения
не требует правок этого пакета.

Мутабельность
-------------

``Node`` и ``OrientedEdge`` неизменяемы. ``RoadGraph`` строит индексы
смежности при создании, поэтому рассчитан на схему «загрузили — много
раз спросили». После правки ``nodes``/``edges`` руками нужно вызвать
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

    #: Произвольные аннотации вершины. Исключены из сравнения: тождество
    #: вершины задаётся идентификатором и координатами, а не аннотациями,
    #: и без этого ``Node`` перестал бы быть хешируемым.
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

    #: Произвольные аннотации ребра: ограничения скорости, тип полосы,
    #: сторона непересекаемой границы — всё, что нужно конкретному алгоритму.
    #: Исключены из сравнения по той же причине, что и у :class:`Node`.
    metadata: dict[str, Any] = field(default_factory=dict[str, Any], compare=False)

    def meta(self, key: str, default: Any = None) -> Any:
        """Значение аннотации или ``default``, если ключа нет."""
        return self.metadata.get(key, default)

    def reversed(self, *, edge_id: int | None = None) -> OrientedEdge:
        """Ребро в обратную сторону с развёрнутой полилинией.

        Аннотации копируются как есть: направление меняет граф, а смысл
        аннотаций знает только тот, кто их проставил. Если аннотация
        зависит от направления движения (например, сторона границы),
        разворачивать её обязан владелец правила, а не модель графа.
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

    # Индексы смежности. Без них каждый запрос соседей — полный проход по
    # рёбрам, а спрашивают их в цикле управления на каждом тике.
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
        Не бросает исключение намеренно: вызывающий сам решает, ругаться
        в лог или падать.
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
