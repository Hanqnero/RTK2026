# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Publishes tf odom -> base_link from /odom (for sim when bridge does not publish tf).

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    """Publishes tf from /odom message (parent_frame_id -> child_frame_id)."""

    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "odom")

        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self._handle_odom, 10)
        self.get_logger().info("odom_tf_broadcaster: publishing tf from %s" % self.odom_topic)

    def _handle_odom(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
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
