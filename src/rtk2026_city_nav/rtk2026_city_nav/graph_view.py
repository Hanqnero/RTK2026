"""Отображение графа в RViz с разделением вершин по роли.

``nav2_route`` публикует граф своим ``route_graph``, но роль вершины ему
неизвестна: ``kind`` — аннотация этого пакета, и точки решений выводятся
здесь. Поэтому раскраска по роли публикуется отдельно.

Публикуется один раз с ``transient_local``: граф не меняется в движении,
а подписчик получает последнее сообщение в момент подписки.
"""

from __future__ import annotations

from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from rtk2026_pose_graph.model import RoadGraph

#: Цвет и размер по роли вершины: точка решения и проходная.
_LOGICAL = ((0.15, 0.45, 1.0, 1.0), 0.14)
_PASSTHROUGH = ((0.55, 0.55, 0.55, 0.9), 0.07)

_EDGE_COLOR = (0.9, 0.9, 0.9, 0.45)
_EDGE_WIDTH = 0.02
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

    def publish(self, graph: RoadGraph, decision_points: frozenset[int]) -> None:
        markers = [
            self._vertices("logical", 0, graph, sorted(decision_points), *_LOGICAL),
            self._vertices(
                "passthrough",
                1,
                graph,
                sorted(set(graph.nodes) - decision_points),
                *_PASSTHROUGH,
            ),
            self._edges(graph),
        ]
        # Подпись — отдельный маркер: текстовый маркер один на точку.
        markers += [
            self._label(graph, node_id, index)
            for index, node_id in enumerate(sorted(graph.nodes), start=10)
        ]
        self._publisher.publish(MarkerArray(markers=markers))

    def _marker(self, namespace: str, marker_id: int, kind: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = f"city_nav_graph/{namespace}"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = kind
        return marker

    def _vertices(
        self,
        namespace: str,
        marker_id: int,
        graph: RoadGraph,
        vertices: list[int],
        color: tuple[float, float, float, float],
        size: float,
    ) -> Marker:
        marker = self._marker(namespace, marker_id, Marker.SPHERE_LIST)
        marker.scale.x = marker.scale.y = marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [self._point(graph, v) for v in vertices]
        return marker

    def _edges(self, graph: RoadGraph) -> Marker:
        """Рёбра как есть: граф хранит каждый сегмент один раз."""
        marker = self._marker("edges", 2, Marker.LINE_LIST)
        marker.scale.x = _EDGE_WIDTH
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = _EDGE_COLOR
        for edge in graph.edges.values():
            marker.points.append(self._point(graph, edge.start_id))
            marker.points.append(self._point(graph, edge.end_id))
        return marker

    def _label(self, graph: RoadGraph, node_id: int, marker_id: int) -> Marker:
        marker = self._marker("labels", marker_id, Marker.TEXT_VIEW_FACING)
        marker.scale.z = _LABEL_SIZE
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.pose.position = self._point(graph, node_id, z=_LABEL_LIFT)
        marker.pose.orientation.w = 1.0
        marker.text = str(node_id)
        return marker

    @staticmethod
    def _point(graph: RoadGraph, node_id: int, *, z: float = 0.0) -> Point:
        node = graph.nodes[node_id]
        return Point(x=float(node.x), y=float(node.y), z=z)
