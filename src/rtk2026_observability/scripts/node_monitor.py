#!/usr/bin/env python3
"""Диагностика ROS 2 нод, lifecycle-состояний и событий ``/rosout``.

Состав проверяемых нод хранится во внешнем YAML-файле. Результаты
публикуются через :mod:`diagnostic_updater` в стандартный ``/diagnostics``
и затем попадают в дерево :mod:`diagnostic_aggregator`.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from threading import Lock
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
from lifecycle_msgs.srv import GetState
import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import yaml


def _normalize_ros_name(value: str) -> str:
    """Вернуть абсолютное ROS-имя без повторяющихся разделителей."""

    parts = [part for part in str(value).split("/") if part]

    if not parts:
        return "/"

    return "/" + "/".join(parts)


def _node_full_name(name: str, namespace: str) -> str:
    """Собрать абсолютное имя ноды из ROS graph."""

    return _normalize_ros_name(f"{namespace}/{name}")


@dataclass(frozen=True)
class NodeConfig:
    """Конфигурация одной диагностируемой ROS 2 ноды.

    :param key: Стабильное имя элемента в диагностическом дереве.
    :param node_name: Абсолютное ROS-имя ноды.
    :param required: Считать ли отсутствие ноды ошибкой.
    :param lifecycle: Проверять ли сервис ``get_state``.
    :param expected_state: Ожидаемая строковая метка lifecycle-состояния.
    :param lifecycle_max_age: Максимальный возраст ответа ``get_state``.
    :param expected_services: Сервисы, необходимые для рабочего состояния.
    :param loggers: Имена логгеров, относящихся к этой ноде.
    :param warn_hold_time: Время отображения последнего WARN.
    :param error_hold_time: Время отображения последнего ERROR/FATAL.
    """

    key: str
    node_name: str
    required: bool
    lifecycle: bool
    expected_state: str
    lifecycle_max_age: float
    expected_services: tuple[str, ...]
    loggers: tuple[str, ...]
    warn_hold_time: float
    error_hold_time: float


@dataclass(frozen=True)
class LogEvent:
    """Одно связанное с нодой событие из ``/rosout``."""

    received_ns: int
    level: int
    logger: str
    message: str


@dataclass(frozen=True)
class NodeSnapshot:
    """Потокобезопасный снимок состояния диагностируемой ноды."""

    graph_checked: bool
    present: bool
    available_services: frozenset[str]
    lifecycle_state: str | None
    lifecycle_response_age: float | None
    lifecycle_error: str | None
    events: tuple[LogEvent, ...]
    warn_count: int
    error_count: int


class NodeHealthTask(DiagnosticTask):
    """Одна задача :mod:`diagnostic_updater` для ROS 2 ноды."""

    def __init__(
        self,
        monitor: "NodeMonitor",
        config: NodeConfig,
    ) -> None:
        """Создать задачу диагностики ноды."""

        super().__init__(f"Nodes/{config.key}")
        self._monitor = monitor
        self._config = config

    def run(self, status):
        """Сформировать текущий ``DiagnosticStatus``."""

        snapshot = self._monitor.snapshot(self._config.key)
        now_ns = time.monotonic_ns()

        status.add("Node", self._config.node_name)
        status.add("Required", str(self._config.required))
        status.add("Present in ROS graph", str(snapshot.present))
        status.add(
            "Expected services",
            ", ".join(self._config.expected_services) or "none",
        )
        status.add("WARN since startup", str(snapshot.warn_count))
        status.add("ERROR/FATAL since startup", str(snapshot.error_count))

        if not snapshot.graph_checked:
            status.summary(
                DiagnosticStatus.STALE,
                "ROS graph ещё не проверен",
            )
            return status

        if not snapshot.present:
            if self._config.required:
                status.summary(
                    DiagnosticStatus.ERROR,
                    "Обязательная нода отсутствует в ROS graph",
                )
            else:
                status.summary(
                    DiagnosticStatus.OK,
                    "Необязательная нода не запущена",
                )

            return status

        problems: list[tuple[int, str]] = []

        missing_services = sorted(
            set(self._config.expected_services)
            - set(snapshot.available_services)
        )
        status.add(
            "Missing services",
            ", ".join(missing_services) or "none",
        )

        if missing_services:
            problems.append(
                (
                    DiagnosticStatus.ERROR,
                    "Отсутствуют обязательные сервисы",
                )
            )

        if self._config.lifecycle:
            status.add(
                "Lifecycle expected state",
                self._config.expected_state,
            )
            status.add(
                "Lifecycle current state",
                snapshot.lifecycle_state or "unknown",
            )

            if snapshot.lifecycle_response_age is not None:
                status.add(
                    "Lifecycle response age (s)",
                    f"{snapshot.lifecycle_response_age:.3f}",
                )

            if snapshot.lifecycle_error is not None:
                status.add(
                    "Lifecycle request error",
                    snapshot.lifecycle_error,
                )
                problems.append(
                    (
                        DiagnosticStatus.ERROR,
                        "Ошибка запроса lifecycle-состояния",
                    )
                )
            elif snapshot.lifecycle_state is None:
                problems.append(
                    (
                        DiagnosticStatus.WARN,
                        "Lifecycle-состояние ещё не получено",
                    )
                )
            elif (
                snapshot.lifecycle_response_age is None
                or snapshot.lifecycle_response_age
                > self._config.lifecycle_max_age
            ):
                problems.append(
                    (
                        DiagnosticStatus.STALE,
                        "Ответ lifecycle устарел",
                    )
                )
            elif (
                snapshot.lifecycle_state.lower()
                != self._config.expected_state.lower()
            ):
                problems.append(
                    (
                        DiagnosticStatus.ERROR,
                        "Нода находится в неправильном lifecycle-состоянии",
                    )
                )

        recent_warning: LogEvent | None = None
        recent_error: LogEvent | None = None

        for event in snapshot.events:
            age = (now_ns - event.received_ns) / 1_000_000_000.0

            if (
                event.level >= Log.ERROR
                and age <= self._config.error_hold_time
            ):
                recent_error = event
            elif (
                event.level >= Log.WARN
                and age <= self._config.warn_hold_time
            ):
                recent_warning = event

        if recent_warning is not None:
            status.add("Last WARN logger", recent_warning.logger)
            status.add("Last WARN", recent_warning.message)
            status.add(
                "Last WARN age (s)",
                f"{(now_ns - recent_warning.received_ns) / 1e9:.3f}",
            )

        if recent_error is not None:
            status.add("Last ERROR logger", recent_error.logger)
            status.add("Last ERROR", recent_error.message)
            status.add(
                "Last ERROR age (s)",
                f"{(now_ns - recent_error.received_ns) / 1e9:.3f}",
            )

        if recent_error is not None:
            problems.append(
                (
                    DiagnosticStatus.ERROR,
                    f"Недавний ERROR/FATAL: {recent_error.message}",
                )
            )
        elif recent_warning is not None:
            problems.append(
                (
                    DiagnosticStatus.WARN,
                    f"Недавний WARN: {recent_warning.message}",
                )
            )

        if not problems:
            status.summary(
                DiagnosticStatus.OK,
                "Нода работает нормально",
            )
            return status

        # STALE имеет числовой уровень выше ERROR, поэтому выбираем
        # максимальный стандартный уровень и сохраняем остальные причины.
        level, message = max(problems, key=lambda item: item[0])
        status.summary(level, message)

        if len(problems) > 1:
            status.add(
                "All problems",
                "; ".join(problem for _, problem in problems),
            )

        return status


class NodeMonitor(Node):
    """Диагностировать ROS graph, lifecycle и логи выбранных нод."""

    def __init__(self) -> None:
        """Загрузить YAML и создать диагностические задачи."""

        super().__init__("node_monitor")

        self.declare_parameter("config_file", "")
        self.declare_parameter("graph_check_period", 1.0)

        config_file = str(self.get_parameter("config_file").value)
        graph_check_period = float(
            self.get_parameter("graph_check_period").value
        )

        if not config_file:
            raise ValueError("параметр config_file не задан")

        if graph_check_period <= 0.0:
            raise ValueError(
                "graph_check_period должен быть положительным"
            )

        config_path = Path(config_file)

        if not config_path.is_file():
            raise FileNotFoundError(
                f"файл конфигурации не найден: {config_path}"
            )

        with config_path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)

        nodes = document.get("nodes")

        if not isinstance(nodes, dict) or not nodes:
            raise ValueError(
                "конфигурация должна содержать непустой mapping nodes"
            )

        self._configs = {
            str(key): self._parse_node_config(str(key), values)
            for key, values in nodes.items()
        }

        self._lock = Lock()
        self._graph_checked = False
        self._present_nodes: set[str] = set()
        self._available_services: set[str] = set()
        self._events: dict[str, deque[LogEvent]] = {
            key: deque(maxlen=200)
            for key in self._configs
        }
        self._warn_counts = {
            key: 0
            for key in self._configs
        }
        self._error_counts = {
            key: 0
            for key in self._configs
        }
        self._lifecycle_states: dict[str, str | None] = {
            key: None
            for key in self._configs
        }
        self._lifecycle_response_ns: dict[str, int | None] = {
            key: None
            for key in self._configs
        }
        self._lifecycle_errors: dict[str, str | None] = {
            key: None
            for key in self._configs
        }
        self._lifecycle_futures: dict[str, Any] = {}
        self._lifecycle_clients = {}

        for key, config in self._configs.items():
            if config.lifecycle:
                service_name = (
                    f"{config.node_name.rstrip('/')}/get_state"
                )
                self._lifecycle_clients[key] = self.create_client(
                    GetState,
                    service_name,
                )

        # Получаем только новые события /rosout. История до запуска
        # диагностики здесь не нужна: она будет сохранена через rosbag2.
        rosout_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._rosout_subscription = self.create_subscription(
            Log,
            "/rosout",
            self._rosout_callback,
            rosout_qos,
        )

        self._updater = Updater(self, period=1.0)
        self._updater.setHardwareID("rtk2026_nodes")
        self._tasks = []

        for key, config in self._configs.items():
            task = NodeHealthTask(self, config)
            self._tasks.append(task)
            self._updater.add(task)
            self.get_logger().info(
                f"Мониторинг ноды {config.node_name}, "
                f"lifecycle={config.lifecycle}, "
                f"required={config.required}"
            )

        # Проверка ROS graph и lifecycle выполняется по wall time.
        self._graph_timer = self.create_timer(
            graph_check_period,
            self._refresh_graph,
        )
        self._refresh_graph()

    def _refresh_graph(self) -> None:
        """Обновить список нод, сервисов и lifecycle-состояния."""

        present_nodes = {
            _node_full_name(name, namespace)
            for name, namespace
            in self.get_node_names_and_namespaces()
        }
        available_services = {
            _normalize_ros_name(name)
            for name, _ in self.get_service_names_and_types()
        }

        with self._lock:
            self._present_nodes = present_nodes
            self._available_services = available_services
            self._graph_checked = True

        for key, client in self._lifecycle_clients.items():
            config = self._configs[key]

            if config.node_name not in present_nodes:
                continue

            current_future = self._lifecycle_futures.get(key)

            if current_future is not None and not current_future.done():
                continue

            if not client.service_is_ready():
                continue

            future = client.call_async(GetState.Request())
            self._lifecycle_futures[key] = future
            future.add_done_callback(
                partial(self._lifecycle_response, key)
            )

    def _lifecycle_response(self, key: str, future: Any) -> None:
        """Сохранить результат асинхронного вызова ``get_state``."""

        try:
            response = future.result()
            state = str(response.current_state.label)
            error = None
        except Exception as exception:  # noqa: BLE001
            state = None
            error = str(exception)

        with self._lock:
            self._lifecycle_states[key] = state
            self._lifecycle_response_ns[key] = time.monotonic_ns()
            self._lifecycle_errors[key] = error

    def _rosout_callback(self, message: Log) -> None:
        """Связать WARN/ERROR из ``/rosout`` с настроенной нодой."""

        if message.level < Log.WARN:
            return

        logger_name = str(message.name).lstrip("/")
        event = LogEvent(
            received_ns=time.monotonic_ns(),
            level=int(message.level),
            logger=logger_name,
            message=str(message.msg),
        )

        for key, config in self._configs.items():
            if not any(
                logger_name == candidate
                or logger_name.startswith(f"{candidate}.")
                for candidate in config.loggers
            ):
                continue

            with self._lock:
                self._events[key].append(event)

                if message.level >= Log.ERROR:
                    self._error_counts[key] += 1
                else:
                    self._warn_counts[key] += 1

    def snapshot(self, key: str) -> NodeSnapshot:
        """Вернуть согласованный снимок состояния одной ноды."""

        config = self._configs[key]
        now_ns = time.monotonic_ns()

        with self._lock:
            response_ns = self._lifecycle_response_ns[key]

            if response_ns is None:
                response_age = None
            else:
                response_age = (
                    now_ns - response_ns
                ) / 1_000_000_000.0

            return NodeSnapshot(
                graph_checked=self._graph_checked,
                present=config.node_name in self._present_nodes,
                available_services=frozenset(
                    self._available_services
                ),
                lifecycle_state=self._lifecycle_states[key],
                lifecycle_response_age=response_age,
                lifecycle_error=self._lifecycle_errors[key],
                events=tuple(self._events[key]),
                warn_count=self._warn_counts[key],
                error_count=self._error_counts[key],
            )

    @staticmethod
    def _parse_node_config(
        key: str,
        values: Any,
    ) -> NodeConfig:
        """Проверить и преобразовать одну секцию YAML."""

        if not isinstance(values, dict):
            raise TypeError(
                f"nodes.{key} должен быть YAML mapping"
            )

        node_name = _normalize_ros_name(str(values["node"]))
        lifecycle = bool(values.get("lifecycle", False))
        lifecycle_max_age = float(
            values.get("lifecycle_max_age", 3.0)
        )
        warn_hold_time = float(values.get("warn_hold_time", 30.0))
        error_hold_time = float(
            values.get("error_hold_time", 120.0)
        )

        if lifecycle_max_age <= 0.0:
            raise ValueError(
                f"nodes.{key}.lifecycle_max_age должен быть положительным"
            )

        if warn_hold_time < 0.0 or error_hold_time < 0.0:
            raise ValueError(
                f"nodes.{key}: времена удержания не могут быть отрицательными"
            )

        expected_services = tuple(
            _normalize_ros_name(str(service))
            for service in values.get("expected_services", [])
        )
        logger_values = values.get(
            "loggers",
            [node_name.rsplit("/", maxsplit=1)[-1]],
        )
        loggers = tuple(
            str(logger).lstrip("/")
            for logger in logger_values
        )

        return NodeConfig(
            key=key,
            node_name=node_name,
            required=bool(values.get("required", True)),
            lifecycle=lifecycle,
            expected_state=str(
                values.get("expected_state", "active")
            ),
            lifecycle_max_age=lifecycle_max_age,
            expected_services=expected_services,
            loggers=loggers,
            warn_hold_time=warn_hold_time,
            error_hold_time=error_hold_time,
        )


def main(args=None) -> None:
    """Запустить универсальный монитор ROS 2 нод."""

    rclpy.init(args=args)
    node = NodeMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
