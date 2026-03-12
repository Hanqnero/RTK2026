# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Adapted from MentorPi peripherals/tf_broadcaster_imu.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class IMUTransformBroadcaster(Node):
    """Publishes tf from imu_link to imu_optical_frame using orientation from /imu."""

    def __init__(self):
        super().__init__("tf_broadcaster_imu")
        self.declare_parameter("imu_link", "imu_link")
        self.declare_parameter("imu_frame", "imu_optical_frame")
        self.declare_parameter("imu_topic", "imu")

        self.target_frame = self.get_parameter("imu_link").get_parameter_value().string_value
        self.imu_frame = self.get_parameter("imu_frame").get_parameter_value().string_value
        self.imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value

        self.trans = TransformStamped()
        self.imu_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, self.imu_topic, self._handle_imu, 1)
        self.get_logger().info(
            "tf_broadcaster_imu: %s -> %s from %s"
            % (self.target_frame, self.imu_frame, self.imu_topic)
        )

    def _handle_imu(self, msg):
        self.trans.header.stamp = self.get_clock().now().to_msg()
        self.trans.header.frame_id = self.target_frame
        self.trans.child_frame_id = self.imu_frame
        self.trans.transform.translation.x = 0.0
        self.trans.transform.translation.y = 0.0
        self.trans.transform.translation.z = 0.0
        self.trans.transform.rotation = msg.orientation
        self.imu_broadcaster.sendTransform(self.trans)


def main(args=None):
    rclpy.init(args=args)
    node = IMUTransformBroadcaster()
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
