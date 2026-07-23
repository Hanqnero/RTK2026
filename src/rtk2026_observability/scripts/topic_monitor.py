#!/usr/bin/env python3
"""Универсальная диагностика ROS 2 topics RTK2026.

Список наблюдаемых topics, их типы, ожидаемые частоты и QoS задаются
во внешнем YAML-файле. Нода ничего не знает о конкретном маршруте,
алгоритме управления или характере движения робота.

Результаты публикуются стандартным :mod:`diagnostic_updater`
в ``/diagnostics``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
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
    qos_profile_system_default,
)
from rosidl_runtime_py.utilities import get_message
import yaml


@dataclass(frozen=True)
class TopicConfig:
    """Конфигурация одной диагностируемой ROS 2 topic.

    :param key: Стабильный идентификатор внутри диагностического дерева.
    :param topic: Полное имя ROS 2 topic.
    :param type_name: Тип в форме ``package/msg/Message``.
    :param required: Считать ли отсутствие сообщений ошибкой.
    :param expected_rate: Номинальная частота. Ноль отключает её проверку.
    :param tolerance: Допустимое относительное отклонение частоты.
    :param window_size: Число timestamp для расчёта частоты.
    :param max_age: Максимальный возраст сообщения. Ноль отключает проверку.
    :param stamp_field: Путь к builtin_interfaces/Time внутри сообщения.
    :param qos: Имя готового QoS-профиля.
    """

    key: str
    topic: str
    type_name: str
    required: bool
    expected_rate: float
    tolerance: float
    window_size: int
    max_age: float
    stamp_field: str | None
    qos: str


def _time_to_nanoseconds(value: Any) -> int:
    """Преобразовать ``builtin_interfaces/Time`` в наносекунды."""

    if not hasattr(value, "sec") or not hasattr(value, "nanosec"):
        raise TypeError("поле не является builtin_interfaces/msg/Time")

    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def _read_stamp(message: Any, field_path: str) -> int:
    """Получить timestamp из сообщения по пути наподобие ``header.stamp``."""

    value = message

    for field_name in field_path.split("."):
        if not hasattr(value, field_name):
            raise AttributeError(
                f"в сообщении отсутствует поле {field_path!r}"
            )

        value = getattr(value, field_name)

    return _time_to_nanoseconds(value)


def _qos_from_name(name: str) -> QoSProfile:
    """Вернуть готовый QoS-профиль по имени из YAML."""

    if name == "sensor_data":
        return qos_profile_sensor_data

    if name == "system_default":
        return qos_profile_system_default

    if name == "transient_local":
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    if name == "best_effort":
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

    if name == "reliable":
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    raise ValueError(f"неизвестный QoS-профиль: {name!r}")


class TopicHealthTask(DiagnosticTask):
    """Одна стандартная задача диагностики ROS 2 topic."""

    def __init__(self, node: Node, config: TopicConfig) -> None:
        """Создать диагностическую задачу."""

        super().__init__(f"Topics/{config.key}")

        self._node = node
        self._config = config
        self._lock = Lock()

        self._message_count = 0
        self._receive_times_ns: deque[int] = deque(
            maxlen=config.window_size
        )
        self._source_times_ns: deque[int] = deque(
            maxlen=config.window_size
        )

        self._last_receive_ns: int | None = None
        self._last_source_stamp_ns: int | None = None
        self._stamp_error: str | None = None

    def tick(self, message: Any) -> None:
        """Зафиксировать получение очередного сообщения."""

        receive_time_ns = self._node.get_clock().now().nanoseconds
        source_stamp_ns: int | None = None
        stamp_error: str | None = None

        if self._config.stamp_field is not None:
            try:
                source_stamp_ns = _read_stamp(
                    message,
                    self._config.stamp_field,
                )

                if source_stamp_ns == 0:
                    stamp_error = "получен нулевой timestamp"

            except (AttributeError, TypeError) as exception:
                stamp_error = str(exception)

        with self._lock:
            self._message_count += 1
            self._last_receive_ns = receive_time_ns
            self._receive_times_ns.append(receive_time_ns)
            self._stamp_error = stamp_error

            if source_stamp_ns is not None:
                # При сбросе Gazebo время может начать отсчёт заново.
                # В этом случае старое окно больше нельзя использовать.
                if (
                    self._last_source_stamp_ns is not None
                    and source_stamp_ns < self._last_source_stamp_ns
                ):
                    self._source_times_ns.clear()
                self._source_times_ns.append(source_stamp_ns)

    @staticmethod
    def _calculate_rate(timestamps_ns: list[int]) -> float | None:
        """Рассчитать среднюю частоту по timestamp окна."""

        if len(timestamps_ns) < 2:
            return None

        duration_ns = timestamps_ns[-1] - timestamps_ns[0]

        if duration_ns <= 0:
            return None

        return (
            (len(timestamps_ns) - 1)
            * 1_000_000_000.0
            / duration_ns
        )

    def run(self, status):
        """Сформировать стандартный ``DiagnosticStatus``."""

        now_ns = self._node.get_clock().now().nanoseconds

        with self._lock:
            message_count = self._message_count
            receive_times_ns = list(self._receive_times_ns)
            source_times_ns = list(self._source_times_ns)
            last_receive_ns = self._last_receive_ns
            last_source_stamp_ns = self._last_source_stamp_ns
            stamp_error = self._stamp_error

        status.add("Topic", self._config.topic)
        status.add("Type", self._config.type_name)
        status.add("Required", str(self._config.required))
        status.add("QoS profile", self._config.qos)
        status.add("Messages since startup", str(message_count))

        if message_count == 0:
            if self._config.required:
                status.summary(
                    DiagnosticStatus.STALE,
                    "Обязательная topic не публикует сообщения",
                )
            else:
                status.summary(
                    DiagnosticStatus.OK,
                    "Необязательная topic пока неактивна",
                )

            return status

        problems: list[tuple[int, str]] = []

        if stamp_error is not None:
            problems.append(
                (
                    DiagnosticStatus.ERROR,
                    f"Некорректный timestamp: {stamp_error}",
                )
            )

        # Для stamped-сообщений частота рассчитывается по времени источника.
        # Поэтому замедление Gazebo относительно wall time не создаёт WARN.
        if len(source_times_ns) >= 2:
            rate = self._calculate_rate(source_times_ns)
            rate_reference = "message timestamp"
        else:
            rate = self._calculate_rate(receive_times_ns)
            rate_reference = "receive time"

        if rate is not None:
            status.add("Actual frequency (Hz)", f"{rate:.3f}")
            status.add("Frequency reference", rate_reference)

        if self._config.expected_rate > 0.0:
            minimum_rate = (
                self._config.expected_rate
                * (1.0 - self._config.tolerance)
            )
            maximum_rate = (
                self._config.expected_rate
                * (1.0 + self._config.tolerance)
            )

            status.add(
                "Target frequency (Hz)",
                f"{self._config.expected_rate:.3f}",
            )
            status.add(
                "Minimum acceptable frequency (Hz)",
                f"{minimum_rate:.3f}",
            )
            status.add(
                "Maximum acceptable frequency (Hz)",
                f"{maximum_rate:.3f}",
            )

            if rate is None:
                problems.append(
                    (
                        DiagnosticStatus.WARN,
                        "Недостаточно сообщений для расчёта частоты",
                    )
                )
            elif rate < minimum_rate:
                problems.append(
                    (
                        DiagnosticStatus.WARN,
                        "Частота ниже допустимой",
                    )
                )
            elif rate > maximum_rate:
                problems.append(
                    (
                        DiagnosticStatus.WARN,
                        "Частота выше допустимой",
                    )
                )

        if self._config.max_age > 0.0:
    # Свежесть topic проверяется по времени получения сообщения.
    #
    # Это wall time самой диагностической ноды. Поэтому проверка
    # одинаково работает в симуляции, на rosbag и на реальном роботе
    # и не требует подписки topic_monitor на высокочастотный /clock.
            if last_receive_ns is not None:
                age_seconds = (
                    now_ns - last_receive_ns
                ) / 1_000_000_000.0
                age_reference = "receive time"
            else:
                age_seconds = float("inf")
                age_reference = "unknown"

            status.add("Message age (s)", f"{age_seconds:.6f}")
            status.add("Age reference", age_reference)
            status.add(
                "Maximum acceptable age (s)",
                f"{self._config.max_age:.6f}",
            )

            if age_seconds < -0.05:
                problems.append(
                    (
                        DiagnosticStatus.ERROR,
                        "Timestamp сообщения находится в будущем",
                    )
                )
            elif age_seconds > self._config.max_age:
                problems.append(
                    (
                        DiagnosticStatus.STALE,
                        "Последнее сообщение устарело",
                    )
                )

        if not problems:
            status.summary(
                DiagnosticStatus.OK,
                "Topic работает нормально",
            )
            return status

        # Выбираем наиболее серьёзную найденную проблему.
        level, message = max(problems, key=lambda item: item[0])
        status.summary(level, message)

        if len(problems) > 1:
            status.add(
                "All problems",
                "; ".join(problem for _, problem in problems),
            )

        return status


class TopicMonitorNode(Node):
    """Диагностировать произвольный набор ROS 2 topics."""

    def __init__(self) -> None:
        """Загрузить YAML и создать динамические subscriptions."""

        super().__init__("topic_monitor")

        self.declare_parameter("config_file", "")
        config_file = str(self.get_parameter("config_file").value)

        if not config_file:
            raise ValueError("параметр config_file не задан")

        config_path = Path(config_file)

        if not config_path.is_file():
            raise FileNotFoundError(
                f"файл конфигурации не найден: {config_path}"
            )

        with config_path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)

        topics = document.get("topics")

        if not isinstance(topics, dict) or not topics:
            raise ValueError(
                "конфигурация должна содержать непустой mapping topics"
            )

        self._updater = Updater(self, period=1.0)
        self._updater.setHardwareID("rtk2026_topics")

        # Сохраняем subscriptions, чтобы они не были удалены сборщиком мусора.
        self._subscriptions = []
        self._tasks = []

        for key, values in topics.items():
            topic_config = self._parse_topic_config(key, values)

            message_type = get_message(topic_config.type_name)
            qos_profile = _qos_from_name(topic_config.qos)

            task = TopicHealthTask(self, topic_config)

            subscription = self.create_subscription(
                message_type,
                topic_config.topic,
                task.tick,
                qos_profile,
            )

            self._tasks.append(task)
            self._subscriptions.append(subscription)
            self._updater.add(task)

            self.get_logger().info(
                f"Мониторинг {topic_config.topic} "
                f"[{topic_config.type_name}], "
                f"required={topic_config.required}"
            )

    @staticmethod
    def _parse_topic_config(
        key: str,
        values: Any,
    ) -> TopicConfig:
        """Проверить и преобразовать одну секцию YAML."""

        if not isinstance(values, dict):
            raise TypeError(
                f"topics.{key} должен быть YAML mapping"
            )

        topic = str(values["topic"])
        type_name = str(values["type"])

        expected_rate = float(values.get("expected_rate", 0.0))
        tolerance = float(values.get("tolerance", 0.2))
        window_size = int(values.get("window_size", 20))
        max_age = float(values.get("max_age", 0.0))

        if expected_rate < 0.0:
            raise ValueError(
                f"topics.{key}.expected_rate не может быть отрицательным"
            )

        if not 0.0 <= tolerance < 1.0:
            raise ValueError(
                f"topics.{key}.tolerance должна быть в диапазоне [0, 1)"
            )

        if window_size < 2:
            raise ValueError(
                f"topics.{key}.window_size должна быть не меньше 2"
            )

        if max_age < 0.0:
            raise ValueError(
                f"topics.{key}.max_age не может быть отрицательным"
            )

        stamp_field_value = values.get("stamp_field")
        stamp_field = (
            str(stamp_field_value)
            if stamp_field_value is not None
            else None
        )

        return TopicConfig(
            key=str(key),
            topic=topic,
            type_name=type_name,
            required=bool(values.get("required", True)),
            expected_rate=expected_rate,
            tolerance=tolerance,
            window_size=window_size,
            max_age=max_age,
            stamp_field=stamp_field,
            qos=str(values.get("qos", "system_default")),
        )


def main(args=None) -> None:
    """Запустить универсальный монитор ROS 2 topics."""

    rclpy.init(args=args)
    node = TopicMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()