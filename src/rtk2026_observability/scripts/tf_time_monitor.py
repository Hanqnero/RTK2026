#!/usr/bin/env python3
"""Диагностика TF-дерева и согласованности времени RTK2026.

Нода намеренно работает по wall time. Она не подписывается на высокочастотный
``/clock`` и поэтому почти не влияет на загрузку симуляции. Timestamp сообщений
используются только для поиска нулевых значений, скачков назад и рассинхронизации
потоков.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from threading import Lock
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage
import yaml


def _stamp_ns(message: Any) -> int:
    """Вернуть ``header.stamp`` сообщения в наносекундах."""

    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def _yaw(quaternion: Any) -> float:
    """Получить planar yaw из quaternion."""

    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def _angle_delta(first: float, second: float) -> float:
    """Вернуть кратчайшую разницу двух углов."""

    return math.atan2(math.sin(second - first), math.cos(second - first))


@dataclass
class TransformState:
    """Последнее состояние одной TF с накопленной статистикой."""

    parent: str
    receive_ns: int
    stamp_ns: int
    x: float
    y: float
    yaw: float
    translation_jump: float = 0.0
    yaw_jump: float = 0.0
    backwards_count: int = 0
    parent_change_count: int = 0
    last_stamp_backwards: bool = False


@dataclass
class StreamState:
    """Временное состояние одного stamped-потока."""

    receive_ns: int | None = None
    stamp_ns: int | None = None
    frame_id: str = ""
    zero_count: int = 0
    backwards_count: int = 0
    last_stamp_backwards: bool = False


class TransformTask(DiagnosticTask):
    """Проверка наличия, свежести и скачков одной TF."""

    def __init__(self, monitor: "TfTimeMonitor", config: dict[str, Any]) -> None:
        super().__init__(f"TF/{config['key']}")
        self._monitor = monitor
        self._config = config

    def run(self, status):
        now_ns = time.monotonic_ns()
        parent = self._config["parent"]
        child = self._config["child"]
        state = self._monitor.transform_snapshot(child)

        status.add("Expected transform", f"{parent} -> {child}")
        status.add("Required", str(bool(self._config.get("required", True))))

        if state is None:
            level = (
                DiagnosticStatus.STALE
                if self._config.get("required", True)
                else DiagnosticStatus.OK
            )
            status.summary(level, "TF не получена")
            return status

        age = (now_ns - state.receive_ns) / 1e9
        status.add("Actual parent", state.parent)
        status.add("Receive age (s)", f"{age:.3f}")
        status.add("Latest translation jump (m)", f"{state.translation_jump:.4f}")
        status.add("Latest yaw jump (rad)", f"{state.yaw_jump:.4f}")
        status.add("Backwards timestamps", str(state.backwards_count))
        status.add("Parent changes", str(state.parent_change_count))

        problems: list[tuple[int, str]] = []
        if state.parent != parent:
            problems.append((DiagnosticStatus.ERROR, "У TF неверный parent frame"))

        max_age = float(self._config.get("max_age", 0.0))
        if max_age > 0.0 and age > max_age:
            problems.append((DiagnosticStatus.STALE, "Динамическая TF устарела"))

        if state.last_stamp_backwards:
            problems.append((DiagnosticStatus.ERROR, "Timestamp TF шёл назад"))
        if state.parent_change_count:
            problems.append((DiagnosticStatus.ERROR, "У child frame менялся parent"))

        jump_warn = float(self._config.get("translation_jump_warn", 0.0))
        if jump_warn > 0.0 and state.translation_jump > jump_warn:
            problems.append((DiagnosticStatus.WARN, "Обнаружен скачок положения TF"))

        yaw_warn = float(self._config.get("yaw_jump_warn", 0.0))
        if yaw_warn > 0.0 and state.yaw_jump > yaw_warn:
            problems.append((DiagnosticStatus.WARN, "Обнаружен скачок угла TF"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "TF корректна")
        return status


class ChainTask(DiagnosticTask):
    """Проверка связности статической или смешанной TF-цепочки."""

    def __init__(self, monitor: "TfTimeMonitor", config: dict[str, Any]) -> None:
        super().__init__(f"TF/{config['key']}")
        self._monitor = monitor
        self._config = config

    def run(self, status):
        parent = self._config["parent"]
        child = self._config["child"]
        path = self._monitor.find_path(parent, child)
        status.add("Expected chain", f"{parent} -> {child}")
        status.add("Resolved path", " -> ".join(path) if path else "none")
        if path:
            status.summary(DiagnosticStatus.OK, "TF-цепочка связна")
        else:
            level = (
                DiagnosticStatus.ERROR
                if self._config.get("required", True)
                else DiagnosticStatus.OK
            )
            status.summary(level, "TF-цепочка разорвана")
        return status


class TimeTask(DiagnosticTask):
    """Проверка timestamp и frame_id одного ROS-потока."""

    def __init__(self, monitor: "TfTimeMonitor", config: dict[str, Any]) -> None:
        super().__init__(f"Time/{config['key']}")
        self._monitor = monitor
        self._config = config

    def run(self, status):
        state = self._monitor.stream_snapshot(self._config["key"])
        status.add("Topic", self._config["topic"])
        status.add("Expected frame", self._config.get("frame_id", "any"))

        if state.receive_ns is None:
            level = (
                DiagnosticStatus.STALE
                if self._config.get("required", True)
                else DiagnosticStatus.OK
            )
            status.summary(level, "Сообщения пока не получены")
            return status

        age = (time.monotonic_ns() - state.receive_ns) / 1e9
        status.add("Receive age (s)", f"{age:.3f}")
        status.add("Frame", state.frame_id or "empty")
        status.add("Zero timestamps", str(state.zero_count))
        status.add("Backwards timestamps", str(state.backwards_count))

        problems: list[tuple[int, str]] = []
        expected_frame = self._config.get("frame_id", "")
        if expected_frame and state.frame_id != expected_frame:
            problems.append((DiagnosticStatus.ERROR, "Неверный frame_id"))
        if state.zero_count:
            problems.append((DiagnosticStatus.ERROR, "Есть нулевые timestamp"))
        if state.last_stamp_backwards:
            problems.append((DiagnosticStatus.ERROR, "Timestamp шёл назад"))
        max_age = float(self._config.get("max_age", 0.0))
        if max_age > 0.0 and age > max_age:
            problems.append((DiagnosticStatus.STALE, "Поток перестал обновляться"))

        reference_key = self._config.get("compare_with")
        if reference_key and state.stamp_ns is not None:
            reference = self._monitor.stream_snapshot(reference_key)
            if reference.stamp_ns is not None:
                skew = abs(state.stamp_ns - reference.stamp_ns) / 1e9
                status.add(f"Skew to {reference_key} (s)", f"{skew:.4f}")
                max_skew = float(self._config.get("max_skew", 0.0))
                if max_skew > 0.0 and skew > max_skew:
                    problems.append(
                        (DiagnosticStatus.WARN, "Потоки рассинхронизированы")
                    )

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "Время и frame_id корректны")
        return status


class TfTimeMonitor(Node):
    """Собирать TF и временную статистику с ограниченными затратами CPU."""

    def __init__(self) -> None:
        super().__init__("tf_time_monitor")
        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value
        if not config_file:
            raise ValueError("parameter config_file is required")

        with Path(config_file).open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        self._lock = Lock()
        self._transforms: dict[str, TransformState] = {}
        self._streams: dict[str, StreamState] = {}

        dynamic_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage, "/tf", lambda msg: self._on_tf(msg, False), dynamic_qos
        )
        self.create_subscription(
            TFMessage, "/tf_static", lambda msg: self._on_tf(msg, True), static_qos
        )

        self._updater = Updater(self)
        self._updater.setHardwareID("rtk2026-tf-time")

        for item in config.get("transforms", []):
            self._updater.add(TransformTask(self, item))
        for item in config.get("chains", []):
            self._updater.add(ChainTask(self, item))
        for item in config.get("streams", []):
            self._streams[item["key"]] = StreamState()
            message_type = get_message(item["type"])
            qos = (
                qos_profile_sensor_data
                if item.get("qos") == "sensor_data"
                else QoSProfile(depth=10)
            )
            self.create_subscription(
                message_type,
                item["topic"],
                lambda msg, key=item["key"]: self._on_stream(key, msg),
                qos,
            )
            self._updater.add(TimeTask(self, item))

    def _on_tf(self, message: TFMessage, static: bool) -> None:
        receive_ns = time.monotonic_ns()
        with self._lock:
            for transform in message.transforms:
                child = transform.child_frame_id.lstrip("/")
                parent = transform.header.frame_id.lstrip("/")
                stamp_ns = _stamp_ns(transform)
                x = float(transform.transform.translation.x)
                y = float(transform.transform.translation.y)
                yaw = _yaw(transform.transform.rotation)
                previous = self._transforms.get(child)
                state = TransformState(parent, receive_ns, stamp_ns, x, y, yaw)
                if previous is not None:
                    state.backwards_count = previous.backwards_count
                    state.parent_change_count = previous.parent_change_count
                    if previous.parent != parent:
                        state.parent_change_count += 1
                    if not static and stamp_ns < previous.stamp_ns:
                        state.backwards_count += 1
                        state.last_stamp_backwards = True
                    if not static and stamp_ns != previous.stamp_ns:
                        jump = math.hypot(x - previous.x, y - previous.y)
                        state.translation_jump = jump
                        state.yaw_jump = abs(_angle_delta(previous.yaw, yaw))
                # Статическая TF не должна протухать.
                if static:
                    state.receive_ns = 0
                self._transforms[child] = state

    def _on_stream(self, key: str, message: Any) -> None:
        receive_ns = time.monotonic_ns()
        stamp_ns = _stamp_ns(message)
        frame_id = getattr(message.header, "frame_id", "").lstrip("/")
        with self._lock:
            state = self._streams[key]
            if stamp_ns == 0:
                state.zero_count += 1
            if state.stamp_ns is not None and stamp_ns < state.stamp_ns:
                state.backwards_count += 1
                state.last_stamp_backwards = True
            else:
                state.last_stamp_backwards = False
            state.receive_ns = receive_ns
            state.stamp_ns = stamp_ns
            state.frame_id = frame_id

    def transform_snapshot(self, child: str) -> TransformState | None:
        """Вернуть копию состояния TF."""

        with self._lock:
            state = self._transforms.get(child)
            return None if state is None else TransformState(**vars(state))

    def stream_snapshot(self, key: str) -> StreamState:
        """Вернуть копию состояния временного потока."""

        with self._lock:
            return StreamState(**vars(self._streams[key]))

    def find_path(self, parent: str, child: str) -> list[str]:
        """Найти путь от parent к child по последним TF."""

        with self._lock:
            parents = {
                frame: state.parent for frame, state in self._transforms.items()
            }
        reverse_path = [child]
        visited = {child}
        current = child
        while current in parents:
            next_frame = parents[current]
            if next_frame in visited:
                break
            current = next_frame
            reverse_path.append(current)
            if current == parent:
                return list(reversed(reverse_path))
            visited.add(current)
        return []


def main(args=None) -> None:
    """Запустить TF/time monitor."""

    rclpy.init(args=args)
    node = TfTimeMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
