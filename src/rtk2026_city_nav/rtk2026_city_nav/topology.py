"""Топология графа: точки решений и цепочки между ними.

Точка решения — вершина, где есть выбор куда ехать: степень не равна двум.
Роль вершины можно задать явно в ``metadata`` под ключом ``kind``, и тогда
степень игнорируется. Это нужно вершинам, которые геометрически ветвятся,
но решений не принимают.

Цепочка — путь между двумя точками решений, внутри которого их нет.

Внутри цепочки может быть ветвление: проходная вершина с тремя и более
рёбрами обслуживает несколько маршрутов сразу. Например одна вершина в
изломе даёт и путь между двумя точками решений, и путь к третьей. Поэтому
цепочки перебираются как простые пути, а не проходятся в одну сторону.

Если между одной парой точек решений нашлось больше одного пути, геометрия
маневра неоднозначна — это ловит проверка графа, см.
:mod:`rtk2026_city_nav.validate`.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from rtk2026_pose_graph.model import OrientedEdge, RoadGraph

#: Ключ metadata, задающий роль вершины.
KIND_KEY = "kind"

#: Значения ``kind``, перекрывающие вывод по степени.
KIND_DECISION = "junction"
KIND_PASSTHROUGH = "geometry"

#: Предел длины цепочки в вершинах. Нужен не для дорожного графа, где
#: цепочки короткие, а как страховка: перебор простых путей по графу
#: с большой связной областью проходных вершин растёт быстро, и лучше
#: недобрать цепочек, чем зависнуть при загрузке.
_MAX_CHAIN_VERTICES = 64


@dataclass(frozen=True)
class Chain:
    """Путь между двумя точками решений.

    ``vertices`` идёт от ``start`` к ``end`` и включает оба конца.
    ``polyline_xy`` склеена из полилиний рёбер, каждая ориентирована по
    ходу от ``start`` к ``end``.
    """

    start: int
    end: int
    vertices: tuple[int, ...]
    polyline_xy: tuple[tuple[float, float], ...]

    @property
    def interior(self) -> tuple[int, ...]:
        """Вершины между концами; для цепочки из одного ребра пусто."""
        return self.vertices[1:-1]

    def reversed(self) -> Chain:
        """Та же цепочка в обратную сторону."""
        return Chain(
            start=self.end,
            end=self.start,
            vertices=tuple(reversed(self.vertices)),
            polyline_xy=tuple(reversed(self.polyline_xy)),
        )


@dataclass
class Topology:
    """Точки решений и цепочки между ними, выведенные из графа один раз."""

    graph: RoadGraph
    decision_points: frozenset[int]
    chains: tuple[Chain, ...]

    _by_start: dict[int, list[Chain]] = field(
        default_factory=dict[int, list["Chain"]], init=False, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        by_start: dict[int, list[Chain]] = {}
        for chain in self.chains:
            by_start.setdefault(chain.start, []).append(chain)
        self._by_start = by_start

    def chains_from(self, vertex: int) -> list[Chain]:
        """Цепочки, выходящие из точки решения."""
        return list(self._by_start.get(vertex, ()))

    def chains_between(self, start: int, end: int) -> list[Chain]:
        """Цепочки ``start -> end``. Больше одной означает неоднозначность."""
        return [chain for chain in self._by_start.get(start, ()) if chain.end == end]

    def neighbors(self, vertex: int) -> list[int]:
        """Точки решений, достижимые из этой одной цепочкой, без повторов."""
        seen: dict[int, None] = {}
        for chain in self._by_start.get(vertex, ()):
            seen.setdefault(chain.end, None)
        return list(seen)


def is_decision_point(graph: RoadGraph, vertex: int, *, degree: int) -> bool:
    """Принимает ли вершина решения.

    ``metadata[kind]`` перекрывает вывод по степени в обе стороны: вершину
    со степенью два можно объявить точкой решения, а ветвящуюся вершину -
    геометрической.
    """
    node = graph.nodes.get(vertex)
    kind = None if node is None else node.meta(KIND_KEY)

    if kind == KIND_DECISION:
        return True
    if kind == KIND_PASSTHROUGH:
        return False
    return degree != 2


def undirected_degrees(graph: RoadGraph) -> dict[int, int]:
    """Степень каждой вершины без учёта направления рёбер.

    Направление здесь ни при чём: цепочка проходит через вершину независимо
    от того, в какую сторону записаны её рёбра.
    """
    neighbors: dict[int, set[int]] = {}
    for edge in graph.edges.values():
        neighbors.setdefault(edge.start_id, set()).add(edge.end_id)
        neighbors.setdefault(edge.end_id, set()).add(edge.start_id)
    return {vertex: len(adjacent) for vertex, adjacent in neighbors.items()}


def build_topology(graph: RoadGraph) -> Topology:
    """Вывести точки решений и все цепочки между ними."""
    degrees = undirected_degrees(graph)
    decision_points = frozenset(
        vertex
        for vertex in graph.nodes
        if is_decision_point(graph, vertex, degree=degrees.get(vertex, 0))
    )

    adjacency = _undirected_adjacency(graph)
    chains: list[Chain] = []
    for start in sorted(decision_points):
        for first in sorted(adjacency.get(start, ())):
            for walked in _walk(adjacency, decision_points, start, first):
                chains.append(_make_chain(graph, walked))

    return Topology(
        graph=graph,
        decision_points=decision_points,
        chains=tuple(chains),
    )


def _undirected_adjacency(graph: RoadGraph) -> dict[int, set[int]]:
    adjacency: dict[int, set[int]] = {}
    for edge in graph.edges.values():
        adjacency.setdefault(edge.start_id, set()).add(edge.end_id)
        adjacency.setdefault(edge.end_id, set()).add(edge.start_id)
    return adjacency


def _walk(
    adjacency: dict[int, set[int]],
    decision_points: frozenset[int],
    start: int,
    first: int,
) -> list[tuple[int, ...]]:
    """Все пути от точки решения до следующей, не заходя внутрь других.

    В проходной вершине с тремя и более рёбрами путь ветвится: такая вершина
    обслуживает несколько маршрутов, и обойти её в одну сторону значило бы
    потерять все, кроме одного.

    Пути простые, вершины в них не повторяются: иначе перебор ходил бы по
    циклам без конца. Из этого же следует, что цепочка не может вернуться
    в свою начальную точку.

    Пути, оборвавшиеся в вершине без продолжения, отбрасываются: так
    выглядит висячая вершина степени один, не объявленная точкой решения.
    """
    found: list[tuple[int, ...]] = []
    stack: list[tuple[int, ...]] = [(start, first)]

    while stack:
        path = stack.pop()

        if path[-1] in decision_points:
            found.append(path)
            continue

        if len(path) >= _MAX_CHAIN_VERTICES:
            continue

        # Наименьший сосед кладётся последним, чтобы сниматься первым:
        # порядок перебора фиксирован, значит и порядок цепочек тоже.
        for following in sorted(adjacency.get(path[-1], ()), reverse=True):
            if following not in path:
                stack.append((*path, following))

    return found


def _make_chain(graph: RoadGraph, vertices: tuple[int, ...]) -> Chain:
    """Собрать цепочку, ориентировав полилинию каждого ребра по ходу."""
    points: list[tuple[float, float]] = []

    for source, target in zip(vertices, vertices[1:]):
        edge = _directed_edge(graph, source, target)
        segment = edge.polyline_xy

        # Стык: последняя точка предыдущего ребра совпадает с первой этого.
        points.extend(segment[1:] if points else segment)

    return Chain(
        start=vertices[0],
        end=vertices[-1],
        vertices=vertices,
        polyline_xy=tuple(points),
    )


def _directed_edge(graph: RoadGraph, source: int, target: int) -> OrientedEdge:
    """Ребро ``source -> target``: своё либо развёрнутое хранимое.

    Граф хранит каждый сегмент один раз, поэтому половина цепочек проходит
    по рёбрам против записанного направления.
    """
    edge = graph.edge_toward_neighbor(source, target)
    if edge is not None:
        return edge

    back = graph.edge_toward_neighbor(target, source)
    if back is None:
        raise ValueError(f"нет ребра между вершинами {source} и {target}")
    return back.reversed()
