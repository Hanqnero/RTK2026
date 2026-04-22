#!/usr/bin/env python3
"""Adapter over yolo_ros Detection2DArray output for RTK2026 driving logic."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rtk2026_interfaces.msg import DrivingDetection
from vision_msgs.msg import Detection2DArray


@dataclass(frozen=True)
class Candidate:
    class_id: str
    command: str
    confidence: float
    box_area: float


class YoloSignAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_sign_adapter")

        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("output_topic", "/perception/driving_detection")
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("min_box_area", 0.0)
        self.declare_parameter("publish_empty", True)
        self.declare_parameter("stop_duration_sec", 2.0)
        self.declare_parameter("log_every_n_messages", 30)
        self.declare_parameter(
            "route_class_to_command",
            [
                "straight=straight_only",
                "left=left_only",
                "right=right_only",
                "no_left=no_left_turn",
                "no_right=no_right_turn",
                "no_straight=no_straight",
            ],
        )
        self.declare_parameter(
            "stop_class_to_action",
            [
                "bus_stop=bus_stop",
            ],
        )
        self.declare_parameter("bus_classes", ["bus"])

        self._route_class_to_command = self._parse_key_value_mapping("route_class_to_command")
        self._stop_class_to_action = self._parse_key_value_mapping("stop_class_to_action")
        self._bus_classes = {self._normalize_label(v) for v in self.get_parameter("bus_classes").value}
        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._min_box_area = float(self.get_parameter("min_box_area").value)
        self._publish_empty = bool(self.get_parameter("publish_empty").value)
        self._stop_duration_sec = float(self.get_parameter("stop_duration_sec").value)
        self._log_every_n_messages = max(1, int(self.get_parameter("log_every_n_messages").value))
        self._msg_count = 0

        detections_topic = str(self.get_parameter("detections_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(DrivingDetection, output_topic, 10)
        self._subscription = self.create_subscription(
            Detection2DArray,
            detections_topic,
            self._on_detections,
            10,
        )
        self.get_logger().info(
            f"yolo_sign_adapter started. detections_topic={detections_topic}, output_topic={output_topic}"
        )

    def _parse_key_value_mapping(self, parameter_name: str) -> dict[str, str]:
        raw = self.get_parameter(parameter_name).value
        out: dict[str, str] = {}
        for item in raw:
            text = str(item).strip()
            if not text or "=" not in text:
                continue
            key, value = text.split("=", 1)
            key_norm = self._normalize_label(key)
            value_norm = value.strip()
            if key_norm and value_norm:
                out[key_norm] = value_norm
        return out

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._msg_count += 1
        best_route: Optional[Candidate] = None
        best_stop: Optional[Candidate] = None
        best_bus: Optional[Candidate] = None

        for detection in msg.detections:
            candidate = self._best_candidate_for_detection(detection)
            if candidate is None:
                continue
            if candidate.command in self._route_class_to_command.values():
                best_route = self._pick_better(best_route, candidate)
            elif candidate.command in self._stop_class_to_action.values():
                best_stop = self._pick_better(best_stop, candidate)
            elif candidate.command == "bus":
                best_bus = self._pick_better(best_bus, candidate)

        if not self._publish_empty and best_route is None and best_stop is None and best_bus is None:
            return

        out = DrivingDetection()
        out.header = msg.header
        if best_route is not None:
            out.route_command = best_route.command
            out.route_class_id = best_route.class_id
            out.route_confidence = float(best_route.confidence)
            out.route_box_area = float(best_route.box_area)
        if best_stop is not None:
            out.stop_action = best_stop.command
            out.stop_class_id = best_stop.class_id
            out.stop_confidence = float(best_stop.confidence)
            out.stop_box_area = float(best_stop.box_area)
            out.stop_duration_sec = float(self._stop_duration_sec)
        if best_bus is not None:
            out.bus_detected = True
            out.bus_class_id = best_bus.class_id
            out.bus_confidence = float(best_bus.confidence)
            out.bus_box_area = float(best_bus.box_area)

        self._publisher.publish(out)

        if self._msg_count % self._log_every_n_messages == 0:
            self.get_logger().info(
                "driving_detection "
                f"route={out.route_command or 'none'} stop={out.stop_action or 'none'} "
                f"bus={'yes' if out.bus_detected else 'no'}"
            )

    def _best_candidate_for_detection(self, detection) -> Optional[Candidate]:
        area = float(detection.bbox.size_x) * float(detection.bbox.size_y)
        if area < self._min_box_area:
            return None

        best: Optional[Candidate] = None
        for result in detection.results:
            hypothesis = getattr(result, "hypothesis", result)
            class_id = str(getattr(hypothesis, "class_id", "")).strip()
            confidence = float(getattr(hypothesis, "score", 0.0))
            if confidence < self._min_confidence:
                continue
            label_norm = self._normalize_label(class_id)
            command = self._route_class_to_command.get(label_norm)
            if command is None:
                command = self._stop_class_to_action.get(label_norm)
            if command is None and label_norm in self._bus_classes:
                command = "bus"
            if command is None:
                continue
            candidate = Candidate(
                class_id=class_id,
                command=command,
                confidence=confidence,
                box_area=area,
            )
            best = self._pick_better(best, candidate)
        return best

    def _pick_better(self, current: Optional[Candidate], new: Candidate) -> Candidate:
        if current is None:
            return new
        if new.box_area > current.box_area:
            return new
        if new.box_area == current.box_area and new.confidence > current.confidence:
            return new
        return current

    @staticmethod
    def _normalize_label(label: str) -> str:
        return str(label).strip().lower().replace(" ", "_").replace("-", "_")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloSignAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
