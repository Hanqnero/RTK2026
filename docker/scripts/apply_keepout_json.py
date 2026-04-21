#!/usr/bin/env python3
import json
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point32
from nav2_msgs.msg import PolygonObject
from nav2_msgs.srv import AddShapes
from rclpy.node import Node


class KeepoutLoader(Node):
    def __init__(self) -> None:
        super().__init__("keepout_json_loader")
        self.client = self.create_client(AddShapes, "/vector_object_server/add_shapes")

    def load(self, json_path: Path) -> int:
        if not json_path.exists():
            self.get_logger().error(f"File not found: {json_path}")
            return 2

        try:
            data = json.loads(json_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.get_logger().error(f"Invalid JSON: {exc}")
            return 3

        frame_id = data.get("frame_id", "map")
        value = int(data.get("value", 100))
        closed = bool(data.get("closed", True))
        polygons = data.get("polygons", [])
        if not polygons:
            self.get_logger().error("JSON has no polygons.")
            return 4

        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /vector_object_server/add_shapes is unavailable.")
            return 5

        req = AddShapes.Request()
        for poly_cfg in polygons:
            points = poly_cfg.get("points", [])
            if len(points) < 3:
                self.get_logger().warn(f"Skip polygon with <3 points: {poly_cfg.get('name', 'unnamed')}")
                continue
            poly = PolygonObject()
            poly.header.frame_id = frame_id
            poly.closed = closed
            poly.value = value
            for xy in points:
                pt = Point32()
                pt.x = float(xy[0])
                pt.y = float(xy[1])
                pt.z = 0.0
                poly.points.append(pt)
            req.polygons.append(poly)

        if not req.polygons:
            self.get_logger().error("No valid polygons to send.")
            return 6

        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if not fut.done() or fut.result() is None:
            self.get_logger().error("AddShapes call failed or timed out.")
            return 7

        self.get_logger().info(f"Loaded {len(req.polygons)} keepout polygons from {json_path}.")
        return 0


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: apply_keepout_json.py /path/to/keepout_parapets.json")
        return 1

    rclpy.init()
    node = KeepoutLoader()
    try:
        return node.load(Path(sys.argv[1]))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
