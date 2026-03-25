# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from rtk2026_interfaces.msg import EncoderReport


class FakeEncoderNode(Node):
    """Publishes EncoderReport at fixed rate for testing without Arduino."""

    def __init__(self):
        super().__init__("fake_encoder")
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("publish_rate", 10.0)
        topic = self.get_parameter("encoder_report_topic").value
        rate = self.get_parameter("publish_rate").value
        self._left = 0
        self._right = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(EncoderReport, topic, qos)
        self._timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            "Fake encoder publishing on %s at %.1f Hz" % (topic, rate)
        )

    def _timer_cb(self):
        msg = EncoderReport()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.left_count = self._left
        msg.right_count = self._right
        msg.left_speed = 0
        msg.right_speed = 0
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeEncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
