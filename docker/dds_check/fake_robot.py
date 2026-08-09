#!/usr/bin/env python3
"""Имитатор робота: публикует ровно то, что показывает RViz.

Зачем отдельно от talker.py
---------------------------

``talker.py`` проверяет транспорт числами: потери, разброс, late-joiner.
Но ``std_msgs/String`` в RViz не отображается, а глазами хочется увидеть,
что через маршрут действительно едут трансформы, карта и скан.

Здесь публикуется минимальный набор, на котором RViz оживает целиком:

``/tf_static``
    ``base_link -> laser``. TRANSIENT_LOCAL: подписчик, подключившийся
    позже, обязан получить это значение. Ровно этот случай ломается,
    если у DDS Router выставить ``remove-unused-entities: true``.

``/tf``
    ``odom -> base_link``. Робот ездит по окружности, чтобы движение
    было видно и заметны рывки из-за джиттера канала.

``/map``
    ``OccupancyGrid``, TRANSIENT_LOCAL. Публикуется однократно: карта
    должна дойти до RViz, запущенного через несколько минут.

``/scan``
    ``LaserScan`` с BEST_EFFORT, как настоящий лидар. Проверяет, что
    согласование QoS не разваливается на best-effort.

``/odom``
    ``Odometry`` в соответствии с ``/tf``.

Настоящей физики здесь нет и не требуется: задача - проверить транспорт,
а не поведение робота.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

MAP_FRAME = "map"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
LASER_FRAME = "laser"


def latched_qos() -> QoSProfile:
    """QoS карты и статических трансформов."""

    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def sensor_qos() -> QoSProfile:
    """QoS лидара: потеря отдельного скана допустима."""

    return QoSProfile(
        depth=5,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class FakeRobot(Node):
    def __init__(self, rate_hz: float, radius_m: float, period_s: float) -> None:
        super().__init__("fake_robot")

        self._radius = radius_m
        self._period = max(period_s, 1.0)
        self._started = time.monotonic()

        self._tf = TransformBroadcaster(self)
        self._tf_static = StaticTransformBroadcaster(self)

        self._map_publisher = self.create_publisher(
            OccupancyGrid, "/map", latched_qos()
        )
        self._scan_publisher = self.create_publisher(
            LaserScan, "/scan", sensor_qos()
        )
        self._odom_publisher = self.create_publisher(Odometry, "/odom", 10)

        self._publish_static_transform()
        self._publish_map()

        self.create_timer(1.0 / max(rate_hz, 0.1), self._tick)

        self.get_logger().info(f"rate_hz = {rate_hz}")
        self.get_logger().info(f"radius_m = {radius_m}")
        self.get_logger().info("topics = /tf /tf_static /map /scan /odom")

    def _publish_static_transform(self) -> None:
        """Однократная публикация до появления подписчиков.

        Смысл проверки именно в этом: RViz стартует позже и всё равно
        обязан получить трансформ.
        """

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = BASE_FRAME
        transform.child_frame_id = LASER_FRAME
        transform.transform.translation.x = 0.12
        transform.transform.translation.z = 0.20
        transform.transform.rotation.w = 1.0

        self._tf_static.sendTransform(transform)
        self.get_logger().info(f"static_transform = {BASE_FRAME} -> {LASER_FRAME}")

    def _publish_map(self) -> None:
        """Карта публикуется один раз, как это делает SLAM после старта."""

        width, height, resolution = 100, 100, 0.05

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = MAP_FRAME
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = -width * resolution / 2.0
        grid.info.origin.position.y = -height * resolution / 2.0
        grid.info.origin.orientation.w = 1.0

        # Свободное поле с занятой рамкой по периметру: на глаз сразу видно,
        # дошла карта целиком или обрезана.
        cells = [0] * (width * height)
        for x in range(width):
            cells[x] = 100
            cells[(height - 1) * width + x] = 100
        for y in range(height):
            cells[y * width] = 100
            cells[y * width + width - 1] = 100

        grid.data = cells
        self._map_publisher.publish(grid)

        self.get_logger().info(f"map = {width}x{height} @ {resolution} м")

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._started
        angle = 2.0 * math.pi * (elapsed % self._period) / self._period

        x = self._radius * math.cos(angle)
        y = self._radius * math.sin(angle)
        yaw = angle + math.pi / 2.0

        stamp = self.get_clock().now().to_msg()

        # map -> odom оставляем единичным: имитируется робот без коррекции
        # SLAM, а проверяется транспорт, а не локализация.
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = ODOM_FRAME
        transform.child_frame_id = BASE_FRAME
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = yaw_to_quaternion(yaw)
        self._tf.sendTransform(transform)

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = MAP_FRAME
        map_to_odom.child_frame_id = ODOM_FRAME
        map_to_odom.transform.rotation.w = 1.0
        self._tf.sendTransform(map_to_odom)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = ODOM_FRAME
        odometry.child_frame_id = BASE_FRAME
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation = yaw_to_quaternion(yaw)
        odometry.twist.twist.linear.x = 2.0 * math.pi * self._radius / self._period
        odometry.twist.twist.angular.z = 2.0 * math.pi / self._period
        self._odom_publisher.publish(odometry)

        self._publish_scan(stamp)

    def _publish_scan(self, stamp) -> None:
        """Скан по кругу с одной ближней отметкой, чтобы был виден поворот."""

        count = 360

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = LASER_FRAME
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / count
        scan.range_min = 0.05
        scan.range_max = 10.0
        scan.ranges = [
            1.5 if index % 60 else 0.6 for index in range(count)
        ]

        self._scan_publisher.publish(scan)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Публиковать TF, карту, скан и одометрию для проверки RViz"
    )
    parser.add_argument("--rate", type=float, default=10.0, help="Частота, Гц")
    parser.add_argument(
        "--radius", type=float, default=1.0, help="Радиус окружности, м"
    )
    parser.add_argument(
        "--period", type=float, default=20.0, help="Период оборота, секунды"
    )
    arguments = parser.parse_args()

    rclpy.init()
    node = FakeRobot(arguments.rate, arguments.radius, arguments.period)

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
