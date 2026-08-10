"""Отображение графа в RViz с разделением вершин по роли.

``nav2_route`` публикует граф своим ``route_graph``, но роль вершины ему
неизвестна: ``kind`` — аннотация этого пакета, и точки решений выводятся
здесь. Поэтому раскраска по роли публикуется отдельно.

Публикуется один раз с ``transient_local``: граф не меняется в движении,
а подписчик получает последнее сообщение в момент подписки, поэтому RViz
покажет его, когда его ни включи.
"""

from __future__ import annotations

from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from rtk2026_city_nav.topology import Topology

#: Точки решений: здесь выбирают, куда ехать.
_LOGICAL_COLOR = (0.15, 0.45, 1.0, 1.0)
_LOGICAL_SIZE = 0.14

#: Проходные вершины: нужны геометрии, решений не принимают.
_PASSTHROUGH_COLOR = (0.55, 0.55, 0.55, 0.9)
_PASSTHROUGH_SIZE = 0.07

#: Цепочки между точками решений.
_CHAIN_COLOR = (0.9, 0.9, 0.9, 0.45)
_CHAIN_WIDTH = 0.02

_LABEL_SIZE = 0.12
_LABEL_LIFT = 0.18


class GraphView:
    """Публикация графа маркерами."""

    def __init__(self, node: Node, *, frame_id: str, topic: str = "~/graph") -> None:
        self._node = node
        self._frame_id = frame_id
        self._publisher = node.create_publisher(
            MarkerArray,
            topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def publish(self, topology: Topology) -> None:
        self._publisher.publish(self._build(topology))

    def _build(self, topology: Topology) -> MarkerArray:
        graph = topology.graph
        logical = sorted(topology.decision_points)
        passthrough = sorted(set(graph.nodes) - topology.decision_points)

        markers = [
            self._spheres(
                "logical", 0, logical, graph, _LOGICAL_COLOR, _LOGICAL_SIZE
            ),
            self._spheres(
                "passthrough",
                1,
                passthrough,
                graph,
                _PASSTHROUGH_COLOR,
                _PASSTHROUGH_SIZE,
            ),
            self._chains(topology),
        ]

        # Подпись — отдельный маркер на вершину: текстовый маркер один на точку.
        markers.extend(
            self._label(node_id, graph, index)
            for index, node_id in enumerate(sorted(graph.nodes), start=10)
        )

        return MarkerArray(markers=markers)

    def _header(self, marker: Marker, namespace: str, marker_id: int) -> None:
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = f"city_nav_graph/{namespace}"
        marker.id = marker_id
        marker.action = Marker.ADD

    def _spheres(
        self,
        namespace: str,
        marker_id: int,
        vertices: list[int],
        graph,
        color: tuple[float, float, float, float],
        size: float,
    ) -> Marker:
        marker = Marker()
        self._header(marker, namespace, marker_id)
        marker.type = Marker.SPHERE_LIST
        marker.scale.x = marker.scale.y = marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [
            Point(x=float(graph.nodes[v].x), y=float(graph.nodes[v].y), z=0.0)
            for v in vertices
        ]
        return marker

    def _chains(self, topology: Topology) -> Marker:
        marker = Marker()
        self._header(marker, "chains", 2)
        marker.type = Marker.LINE_LIST
        marker.scale.x = _CHAIN_WIDTH
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = _CHAIN_COLOR

        # Каждая цепочка есть в обе стороны, поэтому рисуется одна из пары:
        # линии совпали бы и толщина удвоилась.
        drawn: set[tuple[int, int]] = set()
        for chain in topology.chains:
            key = (min(chain.start, chain.end), max(chain.start, chain.end))
            if key in drawn:
                continue
            drawn.add(key)

            for first, second in zip(chain.polyline_xy, chain.polyline_xy[1:]):
                marker.points.append(Point(x=float(first[0]), y=float(first[1]), z=0.0))
                marker.points.append(
                    Point(x=float(second[0]), y=float(second[1]), z=0.0)
                )

        return marker

    def _label(self, node_id: int, graph, marker_id: int) -> Marker:
        node = graph.nodes[node_id]
        marker = Marker()
        self._header(marker, "labels", marker_id)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.z = _LABEL_SIZE
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.pose.position.x = float(node.x)
        marker.pose.position.y = float(node.y)
        marker.pose.position.z = _LABEL_LIFT
        marker.pose.orientation.w = 1.0
        marker.text = str(node_id)
        return marker
