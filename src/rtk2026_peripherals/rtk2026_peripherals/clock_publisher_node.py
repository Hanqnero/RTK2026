# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Publishes /clock from wall time when Isaac is not running (use_fake_odom); avoids TF_OLD_DATA.

import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def main(args=None):
    rclpy.init(args=args)
    node = Node("clock_publisher")
    # use_sim_time is set to False by launch so this node uses wall clock
    pub = node.create_publisher(Clock, "/clock", 10)
    rate = node.create_rate(50)
    start = time.monotonic()
    node.get_logger().info("Publishing /clock from wall time (for use_fake_odom without Isaac)")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            t = time.monotonic() - start
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            msg = Clock()
            msg.clock.sec = sec
            msg.clock.nanosec = nsec
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
