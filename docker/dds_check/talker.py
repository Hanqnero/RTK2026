#!/usr/bin/env python3
"""Передатчик проверки транспорта: три топика с разными QoS.

Зачем три, а не один
--------------------

Обычный ``ros2 topic pub`` создаёт топик с volatile durability и reliable
reliability. Такой топик не проверяет две вещи, на которых как раз и держится
работа RViz через DDS Router:

``/check/volatile``
    RELIABLE + VOLATILE. Базовый случай: просто идут ли данные.

``/check/latched``
    RELIABLE + TRANSIENT_LOCAL. Так публикуются ``/tf_static`` и карта.
    Подписчик, подключившийся позже, обязан получить последнее значение.
    Именно этот случай ломается, если у DDS Router выставить
    ``remove-unused-entities: true``.

``/check/sensor``
    BEST_EFFORT + VOLATILE, глубина 1. Так публикуются лидар и камеры.
    Проверяет, что согласование QoS не разваливается на best-effort.

В каждом сообщении едут номер и время отправки, чтобы приёмник мог посчитать
пропуски и разброс задержки, а не просто сказать «что-то пришло».
"""

from __future__ import annotations

import argparse
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

VOLATILE_TOPIC = "/check/volatile"
LATCHED_TOPIC = "/check/latched"
SENSOR_TOPIC = "/check/sensor"


def volatile_qos() -> QoSProfile:
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def latched_qos() -> QoSProfile:
    """QoS топиков вроде ``/tf_static`` и карты."""

    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def sensor_qos() -> QoSProfile:
    """QoS датчиков: потеря отдельного пакета допустима."""

    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class Talker(Node):
    def __init__(self, rate_hz: float, tag: str) -> None:
        super().__init__("transport_check_talker")

        self._tag = tag
        self._sequence = 0

        self._volatile = self.create_publisher(String, VOLATILE_TOPIC, volatile_qos())
        self._sensor = self.create_publisher(String, SENSOR_TOPIC, sensor_qos())
        self._latched = self.create_publisher(String, LATCHED_TOPIC, latched_qos())

        # Latched-топик публикуется однократно и до появления подписчиков.
        # Смысл проверки именно в том, чтобы подписчик, запущенный позже,
        # всё равно получил это значение.
        message = String()
        message.data = f"{tag} latched at {time.strftime('%H:%M:%S')}"
        self._latched.publish(message)
        self.get_logger().info(f"latched_published = {message.data}")

        self.create_timer(1.0 / max(rate_hz, 0.1), self._tick)

        self.get_logger().info(f"tag = {tag}")
        self.get_logger().info(f"rate_hz = {rate_hz}")
        self.get_logger().info(
            f"topics = {VOLATILE_TOPIC} {SENSOR_TOPIC} {LATCHED_TOPIC}"
        )

    def _tick(self) -> None:
        # Время берётся монотонное и передаётся как есть: приёмник не сравнивает
        # его со своими часами, а считает разброс, поэтому синхронизация часов
        # между машинами не нужна.
        payload = f"{self._tag} {self._sequence} {time.monotonic():.6f}"

        message = String()
        message.data = payload

        self._volatile.publish(message)
        self._sensor.publish(message)

        self._sequence += 1

        if self._sequence % 50 == 0:
            self.get_logger().info(f"sent = {self._sequence}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Публиковать проверочные топики с разными QoS"
    )
    parser.add_argument(
        "--rate", type=float, default=10.0, help="Частота публикации, Гц"
    )
    parser.add_argument(
        "--tag",
        default=socket.gethostname(),
        help="Метка отправителя, попадает в каждое сообщение",
    )
    arguments = parser.parse_args()

    rclpy.init()
    node = Talker(arguments.rate, arguments.tag)

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
