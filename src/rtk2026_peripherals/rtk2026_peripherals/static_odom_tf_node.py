# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Publishes map->odom and odom->base_link at 50 Hz with stamp=now() when use_fake_odom,
# so TF buffer gets fresh timestamps and TF_OLD_DATA warnings stop.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def main(args=None):
    rclpy.init(args=args)
    node = Node("static_odom_tf_publisher")
    br = StaticTransformBroadcaster(node)

    t_map_odom = TransformStamped()
    t_map_odom.header.frame_id = "map"
    t_map_odom.child_frame_id = "odom"
    t_map_odom.transform.rotation.w = 1.0

    t_odom_base = TransformStamped()
    t_odom_base.header.frame_id = "odom"
    t_odom_base.child_frame_id = "base_link"
    t_odom_base.transform.rotation.w = 1.0

    node.get_logger().info("Publishing static map->odom and odom->base_link (use_fake_odom)")
    try:
        now = node.get_clock().now().to_msg()
        t_map_odom.header.stamp = now
        t_odom_base.header.stamp = now
        br.sendTransform([t_map_odom, t_odom_base])
        # Keep node alive so static transforms remain available to new subscribers
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
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
