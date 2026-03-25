#!/usr/bin/env python3
"""
cmd_vel_mux — merges lane controller and obstacle avoidance cmd_vel outputs.

Subscribes:
  /cmd_vel_lane   (Twist) — output of control_lane PD controller
  /avoid_control  (Twist) — output of avoid_construction
  /avoid_active   (Bool)  — True when avoidance is active

Publishes:
  /cmd_vel (Twist) — forwarded to diff_drive

Logic: if avoid_active → publish avoid_control, else forward cmd_vel_lane.
Watchdog: publishes zero Twist if no cmd_vel_lane received within timeout.
"""

import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.node import Node


class CmdVelMux(Node):
    def __init__(self):
        super().__init__("lane_path_publisher")

        self.declare_parameter("watchdog_timeout", 0.5)
        self.declare_parameter("use_avoid_merge", True)

        self._lock = threading.Lock()
        self._lane_twist: Twist | None = None
        self._avoid_twist: Twist | None = None
        self._avoid_active: bool = False
        self._last_lane_time: float = 0.0

        self.sub_lane = self.create_subscription(Twist, "/control/cmd_vel", self._cb_lane, 10)
        self.sub_avoid = self.create_subscription(Twist, "/avoid_control", self._cb_avoid, 10)
        self.sub_active = self.create_subscription(Bool, "/avoid_active", self._cb_active, 10)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self._timer = self.create_timer(0.05, self._timer_cb)  # 20 Hz

        self.get_logger().info("cmd_vel_mux started")

    def _cb_lane(self, msg: Twist):
        with self._lock:
            self._lane_twist = msg
            self._last_lane_time = time.monotonic()

    def _cb_avoid(self, msg: Twist):
        with self._lock:
            self._avoid_twist = msg

    def _cb_active(self, msg: Bool):
        with self._lock:
            self._avoid_active = msg.data

    def _timer_cb(self):
        use_avoid = self.get_parameter("use_avoid_merge").value
        timeout = self.get_parameter("watchdog_timeout").value

        with self._lock:
            active = self._avoid_active
            avoid = self._avoid_twist
            lane = self._lane_twist
            last_t = self._last_lane_time

        if use_avoid and active and avoid is not None:
            self.pub_cmd.publish(avoid)
            return

        if lane is None or (time.monotonic() - last_t) > timeout:
            self.pub_cmd.publish(Twist())
            return

        self.pub_cmd.publish(lane)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
