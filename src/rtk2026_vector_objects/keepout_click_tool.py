#!/usr/bin/env python3
import time
from typing import List

import rclpy
from geometry_msgs.msg import Point32, PointStamped
from nav2_msgs.msg import PolygonObject
from nav2_msgs.srv import AddShapes
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class KeepoutClickTool(Node):
    def __init__(self) -> None:
        super().__init__("keepout_click_tool")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("add_shapes_service", "/vector_object_server/add_shapes")
        self.declare_parameter("polygon_value", 100)
        self.declare_parameter("frame_id", "map")

        clicked_topic = self.get_parameter("clicked_point_topic").get_parameter_value().string_value
        add_shapes_service = self.get_parameter("add_shapes_service").get_parameter_value().string_value
        self._polygon_value = int(self.get_parameter("polygon_value").value)
        self._default_frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self._points: List[PointStamped] = []
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            PointStamped, clicked_topic, self._on_clicked_point, 20, callback_group=self._cb_group
        )
        self._add_shapes_cli = self.create_client(
            AddShapes, add_shapes_service, callback_group=self._cb_group
        )
        self.create_service(Trigger, "~/commit", self._on_commit, callback_group=self._cb_group)
        self.create_service(Trigger, "~/clear", self._on_clear, callback_group=self._cb_group)
        self.create_service(Trigger, "~/undo", self._on_undo, callback_group=self._cb_group)

        self.get_logger().info(
            "Keepout click tool ready. "
            "Click points in RViz PublishPoint tool, then call ~/commit service."
        )

    def _on_clicked_point(self, msg: PointStamped) -> None:
        self._points.append(msg)
        self.get_logger().info(
            f"Point #{len(self._points)}: x={msg.point.x:.3f}, y={msg.point.y:.3f}, frame={msg.header.frame_id or self._default_frame_id}"
        )

    def _on_clear(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        count = len(self._points)
        self._points.clear()
        res.success = True
        res.message = f"Cleared {count} points."
        self.get_logger().info(res.message)
        return res

    def _on_undo(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if not self._points:
            res.success = False
            res.message = "No points to undo."
            return res
        removed = self._points.pop()
        res.success = True
        res.message = f"Removed point: x={removed.point.x:.3f}, y={removed.point.y:.3f}. Remaining: {len(self._points)}"
        self.get_logger().info(res.message)
        return res

    def _on_commit(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if len(self._points) < 3:
            res.success = False
            res.message = "Need at least 3 clicked points to commit polygon."
            return res

        if not self._add_shapes_cli.wait_for_service(timeout_sec=2.0):
            res.success = False
            res.message = "Service /vector_object_server/add_shapes is unavailable."
            return res

        frame_id = self._points[-1].header.frame_id or self._default_frame_id
        poly = PolygonObject()
        poly.header.frame_id = frame_id
        poly.closed = True
        poly.value = self._polygon_value
        poly.points = []
        for p in self._points:
            pt = Point32()
            pt.x = float(p.point.x)
            pt.y = float(p.point.y)
            pt.z = 0.0
            poly.points.append(pt)

        req = AddShapes.Request()
        req.polygons = [poly]
        future = self._add_shapes_cli.call_async(req)
        timeout_at = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < timeout_at:
            time.sleep(0.05)

        if not future.done() or future.result() is None:
            res.success = False
            res.message = "AddShapes call failed or timed out."
            return res

        count = len(self._points)
        self._points.clear()
        res.success = True
        res.message = f"Committed keepout polygon with {count} points."
        self.get_logger().info(res.message)
        return res


def main() -> None:
    rclpy.init()
    node = KeepoutClickTool()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
