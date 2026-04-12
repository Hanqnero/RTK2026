#!/usr/bin/env python3
"""
Lane Mission Controller — sign-triggered active turn + lane resume.

When a direction sign is detected, the node takes over cmd_vel:
  1. TURN phase: publishes a fixed turn command for `turn_duration_sec`
  2. RESUME phase: returns to lane following (passes /control/cmd_vel through)

This is more reliable than just clamping angular.z, because the PD regulator
at an intersection outputs ~0 (straight) regardless of constraints.

Topics:
  Subscribe:
    /control/cmd_vel     (Twist) — raw output from control_lane PD regulator
    /detect/traffic_sign (UInt8) — 1=intersection, 2=left, 3=right
    /avoid_active        (Bool)  — suspend mission during obstacle avoidance
  Publish:
    /cmd_vel             (Twist) — final velocity (mission turn or lane following)

Parameters:
  turn_duration_sec  (float, 3.0)   — how long to actively turn after sign detected
  turn_angular_z     (float, -0.5)  — angular.z during right turn (negative=right in ROS)
  turn_linear_x      (float, 0.02)  — linear.x during active turn (slow)
  vote_window        (int,   10)    — sliding window for sign voting
  vote_threshold     (float, 0.5)   — fraction needed to activate
  allowed_signs      (str,  "all")  — "all" | "right" | "left"
  cooldown_sec       (float, 15.0)  — min seconds between consecutive activations
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt8

SIGN_INTERSECTION = 1
SIGN_LEFT  = 2
SIGN_RIGHT = 3

STATE_NORMAL    = "NORMAL"
STATE_TURNING   = "TURNING"   # actively publishing turn command


class LaneMissionNode(Node):
    def __init__(self):
        super().__init__("lane_mission")

        self.declare_parameter("turn_duration_sec",  3.0)
        self.declare_parameter("turn_angular_z",    -0.5)   # negative = right
        self.declare_parameter("turn_linear_x",      0.02)
        self.declare_parameter("vote_window",        10)
        self.declare_parameter("vote_threshold",      0.5)
        self.declare_parameter("allowed_signs",      "all")
        self.declare_parameter("cooldown_sec",       15.0)

        self._turn_dur    = self.get_parameter("turn_duration_sec").value
        self._turn_az     = self.get_parameter("turn_angular_z").value
        self._turn_lx     = self.get_parameter("turn_linear_x").value
        self._vote_win    = self.get_parameter("vote_window").value
        self._vote_thr    = self.get_parameter("vote_threshold").value
        self._allowed     = self.get_parameter("allowed_signs").value
        self._cooldown    = self.get_parameter("cooldown_sec").value

        self._state        = STATE_NORMAL
        self._turn_end     = 0.0
        self._last_active  = 0.0   # last time a turn was triggered (cooldown)
        self._sign_buf: list[int] = []
        self._avoid_active = False
        self._last_lane_cmd = Twist()  # last cmd from control_lane

        self.create_subscription(Twist, "/control/cmd_vel",     self._cb_cmd_vel, 1)
        self.create_subscription(UInt8, "/detect/traffic_sign", self._cb_sign,    10)
        self.create_subscription(Bool,  "/avoid_active",        self._cb_avoid,   10)

        self._pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.create_timer(0.05, self._tick)   # 20 Hz control loop

        self.get_logger().info("lane_mission started (NORMAL)")

    # ------------------------------------------------------------------
    def _cb_avoid(self, msg: Bool):
        self._avoid_active = msg.data

    def _cb_cmd_vel(self, msg: Twist):
        self._last_lane_cmd = msg
        # In NORMAL mode publish immediately (low latency lane following)
        if self._state == STATE_NORMAL and not self._avoid_active:
            self._pub.publish(msg)

    def _cb_sign(self, msg: UInt8):
        sign = int(msg.data)
        if sign not in (SIGN_LEFT, SIGN_RIGHT, SIGN_INTERSECTION):
            return
        if self._allowed == "right" and sign != SIGN_RIGHT:
            return
        if self._allowed == "left" and sign != SIGN_LEFT:
            return

        self._sign_buf.append(sign)
        if len(self._sign_buf) > self._vote_win:
            self._sign_buf = self._sign_buf[-self._vote_win:]

        if len(self._sign_buf) >= self._vote_win:
            count = self._sign_buf.count(sign)
            if count / self._vote_win >= self._vote_thr:
                self._try_activate(sign)

    def _try_activate(self, sign: int):
        now = time.time()
        if self._state == STATE_TURNING:
            return   # already turning
        if now - self._last_active < self._cooldown:
            return   # cooldown: don't re-trigger immediately after a turn

        # Decide turn direction
        if sign == SIGN_RIGHT:
            self._turn_az = -abs(self._turn_az)   # negative = right
        elif sign == SIGN_LEFT:
            self._turn_az =  abs(self._turn_az)   # positive = left
        else:
            return  # intersection sign alone doesn't trigger a turn

        self._state       = STATE_TURNING
        self._turn_end    = now + self._turn_dur
        self._last_active = now
        self._sign_buf.clear()
        self.get_logger().info(
            f"Sign {sign} → TURNING {'right' if self._turn_az < 0 else 'left'} "
            f"for {self._turn_dur:.1f}s "
            f"(az={self._turn_az:.2f}, lx={self._turn_lx:.3f})"
        )

    # ------------------------------------------------------------------
    def _tick(self):
        now = time.time()

        if self._state == STATE_TURNING:
            if now > self._turn_end:
                self._state = STATE_NORMAL
                self.get_logger().info("Turn complete → NORMAL (lane following)")
                return
            # Publish active turn command
            if self._avoid_active:
                return   # avoidance takes priority
            t = Twist()
            t.linear.x  = self._turn_lx
            t.angular.z = self._turn_az
            self._pub.publish(t)


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
