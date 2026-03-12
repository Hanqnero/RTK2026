# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Publishes a minimal valid /map (OccupancyGrid) when use_fake_odom so global_costmap
# does not reject "malformed" initial messages from slam_toolbox.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy


def main(args=None):
    rclpy.init(args=args)
    node = Node("static_map_publisher")
    qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )
    pub = node.create_publisher(OccupancyGrid, "/map", qos)
    rate = node.create_rate(1.0)

    # 20x20 m at 0.05 m/cell = 400x400, all free (0)
    width = 400
    height = 400
    resolution = 0.05
    origin_x = -10.0
    origin_y = -10.0

    msg = OccupancyGrid()
    msg.header = Header()
    msg.header.frame_id = "map"
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin = Pose()
    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = origin_y
    msg.info.origin.position.z = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * (width * height)

    node.get_logger().info(
        "Publishing static /map %dx%d m (all free) for use_fake_odom"
        % (int(width * resolution), int(height * resolution))
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rate.sleep()
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
