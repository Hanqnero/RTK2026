#!/usr/bin/env python3
import json
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import Point32, PointStamped
from nav2_msgs.msg import PolygonObject
from nav2_msgs.srv import AddShapes
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

#: Версия формата файла зон.
ZONES_FORMAT_VERSION = 1


class KeepoutClickTool(Node):
    def __init__(self) -> None:
        super().__init__("keepout_click_tool")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("add_shapes_service", "/vector_object_server/add_shapes")
        self.declare_parameter("polygon_value", 100)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("zones_path", "")

        clicked_topic = self.get_parameter("clicked_point_topic").get_parameter_value().string_value
        add_shapes_service = self.get_parameter("add_shapes_service").get_parameter_value().string_value
        self._polygon_value = int(self.get_parameter("polygon_value").value)
        self._default_frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        zones_path = self.get_parameter("zones_path").get_parameter_value().string_value.strip()
        self._zones_path = Path(zones_path) if zones_path else None

        self._points: List[PointStamped] = []
        # Замкнутые зоны: они и пишутся в файл, и заново отдаются серверу
        # при следующем запуске.
        self._zones: List[Dict[str, Any]] = []
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
        self.create_service(Trigger, "~/save", self._on_save, callback_group=self._cb_group)
        self.create_service(Trigger, "~/drop_zones", self._on_drop_zones, callback_group=self._cb_group)

        self.get_logger().info(
            "Keepout click tool ready. "
            "Click points in RViz PublishPoint tool, then call ~/commit service."
        )

        self._load_zones()
        if self._zones:
            # Сервер поднимается через lifecycle и на этот момент обычно ещё
            # не активен, поэтому отправка повторяется таймером до успеха.
            self._restore_timer = self.create_timer(
                1.0, self._restore_zones, callback_group=self._cb_group
            )

    # -- Файл зон ---------------------------------------------------------

    def _load_zones(self) -> None:
        if self._zones_path is None:
            self.get_logger().info("zones_path is not set: zones will not persist.")
            return
        if not self._zones_path.is_file():
            self.get_logger().info(f"No zones file yet at {self._zones_path}.")
            return

        try:
            data = json.loads(self._zones_path.read_text(encoding="utf-8"))
            version = int(data.get("version", 0))
            if version != ZONES_FORMAT_VERSION:
                raise ValueError(f"zones format {version}, expected {ZONES_FORMAT_VERSION}")
            zones = [z for z in data.get("zones", []) if len(z.get("points", [])) >= 3]
        except (OSError, ValueError, TypeError) as error:
            self.get_logger().error(f"Zones file not loaded: {error}")
            return

        self._zones = zones
        self.get_logger().info(f"Loaded {len(zones)} zones from {self._zones_path}.")

    def _save_zones(self) -> bool:
        if self._zones_path is None:
            return False

        try:
            self._zones_path.parent.mkdir(parents=True, exist_ok=True)
            self._zones_path.write_text(
                json.dumps(
                    {"version": ZONES_FORMAT_VERSION, "zones": self._zones},
                    ensure_ascii=False,
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
        except OSError as error:
            self.get_logger().error(f"Zones file not written: {error}")
            return False

        return True

    def _restore_zones(self) -> None:
        """Отдать серверу зоны, прочитанные из файла."""
        if not self._add_shapes_cli.wait_for_service(timeout_sec=0.5):
            return

        self._restore_timer.cancel()
        polygons = [self._to_polygon(z) for z in self._zones]
        if self._send(polygons):
            self.get_logger().info(f"Restored {len(polygons)} zones to the server.")
        else:
            self.get_logger().error("Restoring zones failed; server rejected AddShapes.")

    def _to_polygon(self, zone: Dict[str, Any]) -> PolygonObject:
        poly = PolygonObject()
        poly.header.frame_id = str(zone.get("frame_id", self._default_frame_id))
        poly.closed = True
        poly.value = int(zone.get("value", self._polygon_value))
        poly.points = []
        for x, y in zone["points"]:
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            poly.points.append(pt)
        return poly

    def _send(self, polygons: List[PolygonObject]) -> bool:
        """Отправить полигоны серверу и дождаться ответа."""
        req = AddShapes.Request()
        req.polygons = polygons
        future = self._add_shapes_cli.call_async(req)

        timeout_at = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < timeout_at:
            time.sleep(0.05)

        return future.done() and future.result() is not None

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

        zone = {
            "frame_id": self._points[-1].header.frame_id or self._default_frame_id,
            "value": self._polygon_value,
            "points": [
                [round(float(p.point.x), 4), round(float(p.point.y), 4)]
                for p in self._points
            ],
        }

        if not self._send([self._to_polygon(zone)]):
            res.success = False
            res.message = "AddShapes call failed or timed out."
            return res

        count = len(self._points)
        self._points.clear()
        self._zones.append(zone)

        # Пишется сразу: зону отметили — она сохранена, отдельного шага нет.
        saved = self._save_zones()
        where = f" Saved to {self._zones_path}." if saved else ""
        if self._zones_path is not None and not saved:
            where = " NOT saved, see the error above."

        res.success = True
        res.message = f"Committed keepout polygon with {count} points.{where}"
        self.get_logger().info(res.message)
        return res

    def _on_save(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        if self._zones_path is None:
            res.success = False
            res.message = "zones_path is not set: nowhere to save."
            return res

        res.success = self._save_zones()
        res.message = (
            f"Saved {len(self._zones)} zones to {self._zones_path}."
            if res.success
            else "Saving zones failed, see the error above."
        )
        self.get_logger().info(res.message)
        return res

    def _on_drop_zones(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """Забыть все зоны и очистить файл.

        Сервер о них не узнает: убрать фигуру из него нечем, поэтому его
        надо перезапустить.
        """
        count = len(self._zones)
        self._zones.clear()
        self._save_zones()

        res.success = True
        res.message = (
            f"Dropped {count} zones. Restart vector_object_server to clear the mask."
        )
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
