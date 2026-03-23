#!/usr/bin/env python3
"""
Lane Mission Controller — sign-based velocity constraint filter.

Sits between control_lane (/control/cmd_vel) and the robot (/cmd_vel).
When a direction sign is detected, clamps angular.z so the robot can only
turn in the allowed direction — the lane follower then naturally takes the
correct branch at a fork/intersection.

Topics:
  Subscribe:
    /control/cmd_vel    (Twist) — raw output from control_lane PD regulator
    /detect/traffic_sign (UInt8) — 1=intersection, 2=left, 3=right
  Publish:
    /cmd_vel            (Twist) — filtered, constraint-clamped velocity

Parameters:
  constraint_duration_sec (float, 5.0) — how long to hold the constraint after sign is seen
  az_turn_limit          (float, 2.0)  — max |angular.z| during constraint
  vote_window            (int,   10)   — sliding window size for sign voting
  vote_threshold         (float, 0.6)  — fraction of window needed to activate (e.g. 6/10)
  allowed_signs          (str,  "all") — which signs to act on: "all", "right", "left"
  passthrough_avoid      (bool, True)  — pass avoid_control through unclamped when avoid_active
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8


# Sign values from detect_intersection_sign (Enum autovalue: 1-based)
SIGN_INTERSECTION = 1
SIGN_LEFT  = 2
SIGN_RIGHT = 3

# Constraint states
STATE_NORMAL     = "NORMAL"
STATE_LEFT_ONLY  = "LEFT_ONLY"   # angular.z >= 0 (positive = CCW = turn left in ROS)
STATE_RIGHT_ONLY = "RIGHT_ONLY"  # angular.z <= 0 (negative = CW  = turn right in ROS)


class LaneMissionNode(Node):
    def __init__(self):
        super().__init__("lane_mission")

        self.declare_parameter("constraint_duration_sec", 5.0)
        self.declare_parameter("az_turn_limit",           2.0)
        self.declare_parameter("vote_window",             10)
        self.declare_parameter("vote_threshold",          0.6)
        self.declare_parameter("allowed_signs",           "all")
        self.declare_parameter("passthrough_avoid",       True)

        self._duration       = self.get_parameter("constraint_duration_sec").value
        self._az_limit       = self.get_parameter("az_turn_limit").value
        self._vote_window    = self.get_parameter("vote_window").value
        self._vote_threshold = self.get_parameter("vote_threshold").value
        self._allowed        = self.get_parameter("allowed_signs").value  # "all"|"right"|"left"
        self._pass_avoid     = self.get_parameter("passthrough_avoid").value

        self._state          = STATE_NORMAL
        self._constraint_end = 0.0       # wall-clock time when constraint expires
        self._sign_buf: list[int] = []   # sliding vote window

        self._avoid_active   = False

        self.create_subscription(Twist, "/control/cmd_vel",      self._cb_cmd_vel,   1)
        self.create_subscription(UInt8, "/detect/traffic_sign",  self._cb_sign,      10)
        self.create_subscription(Bool,  "/avoid_active",         self._cb_avoid,     10)

        self._pub = self.create_publisher(Twist, "/cmd_vel", 1)

        # Timer: check constraint expiry at 10 Hz
        self.create_timer(0.1, self._tick)

        self.get_logger().info("lane_mission_node started  (NORMAL mode)")

    # ------------------------------------------------------------------
    def _cb_avoid(self, msg: Bool):
        self._avoid_active = msg.data

    def _cb_sign(self, msg: UInt8):
        sign = int(msg.data)
        if sign not in (SIGN_LEFT, SIGN_RIGHT, SIGN_INTERSECTION):
            return

        # Filter by allowed_signs parameter
        if self._allowed == "right" and sign != SIGN_RIGHT:
            return
        if self._allowed == "left" and sign != SIGN_LEFT:
            return

        self._sign_buf.append(sign)
        if len(self._sign_buf) > self._vote_window:
            self._sign_buf = self._sign_buf[-self._vote_window:]

        # Activate when a sign wins the vote (>= vote_threshold fraction of window)
        if len(self._sign_buf) >= self._vote_window:
            count = self._sign_buf.count(sign)
            if count / self._vote_window >= self._vote_threshold:
                self._activate(sign)

    def _activate(self, sign: int):
        now = time.time()
        new_state = (
            STATE_RIGHT_ONLY if sign == SIGN_RIGHT else
            STATE_LEFT_ONLY  if sign == SIGN_LEFT  else
            STATE_NORMAL
        )
        if new_state != self._state:
            self.get_logger().info(
                f"Sign {sign} detected → {new_state} "
                f"(constraint for {self._duration:.1f}s)"
            )
        self._state          = new_state
        self._constraint_end = now + self._duration
        self._sign_buf.clear()

    def _tick(self):
        if self._state != STATE_NORMAL and time.time() > self._constraint_end:
            self.get_logger().info(f"Constraint expired → NORMAL")
            self._state = STATE_NORMAL

    # ------------------------------------------------------------------
    def _cb_cmd_vel(self, msg: Twist):
        # When avoid_construction is active, pass through unclamped (optional)
        if self._avoid_active and self._pass_avoid:
            self._pub.publish(msg)
            return

        filtered = Twist()
        filtered.linear.x  = msg.linear.x
        filtered.linear.y  = msg.linear.y
        filtered.linear.z  = msg.linear.z
        filtered.angular.x = msg.angular.x
        filtered.angular.y = msg.angular.y

        az = msg.angular.z
        if self._state == STATE_RIGHT_ONLY:
            az = min(az, 0.0)                         # clamp to [-limit, 0]
            az = max(az, -self._az_limit)
        elif self._state == STATE_LEFT_ONLY:
            az = max(az, 0.0)                         # clamp to [0, +limit]
            az = min(az, self._az_limit)

        filtered.angular.z = az
        self._pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = LaneMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
