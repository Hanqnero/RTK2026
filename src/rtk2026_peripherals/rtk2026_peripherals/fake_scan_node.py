# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Publishes a minimal LaserScan on /scan for testing without lidar hardware.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScanNode(Node):
    def __init__(self):
        super().__init__("fake_scan")
        self.declare_parameter("frame_id", "lidar_link")
        self.declare_parameter("topic", "scan")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("angle_min", -3.14159)
        self.declare_parameter("angle_max", 3.14159)
        self.declare_parameter("angle_increment", 0.01)

        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        rate = self.get_parameter("rate").get_parameter_value().double_value
        self._range_min = self.get_parameter("range_min").get_parameter_value().double_value
        self._range_max = self.get_parameter("range_max").get_parameter_value().double_value
        self._angle_min = self.get_parameter("angle_min").get_parameter_value().double_value
        self._angle_max = self.get_parameter("angle_max").get_parameter_value().double_value
        self._angle_inc = self.get_parameter("angle_increment").get_parameter_value().double_value

        self._pub = self.create_publisher(LaserScan, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info("fake_scan publishing to %s (frame %s)" % (topic, frame_id))
        self._frame_id = frame_id

    def _publish(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.angle_min = self._angle_min
        msg.angle_max = self._angle_max
        msg.angle_increment = self._angle_inc
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self._range_min
        msg.range_max = self._range_max
        num_readings = int((self._angle_max - self._angle_min) / self._angle_inc)
        msg.ranges = [float("inf")] * num_readings
        msg.intensities = []
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
