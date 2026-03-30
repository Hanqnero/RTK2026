#!/usr/bin/env python3
"""
Lane Mission Controller — YAML-driven sign-triggered trajectory execution.

Чтобы добавить новое действие по знаку — редактируй sign_behaviors.yaml.
Не нужно трогать этот файл.

Topics:
  Subscribe:
    /control/cmd_vel     (Twist) — raw output from control_lane PD regulator
    /detect/traffic_sign (UInt8) — sign ID (1=intersection, 2=left, 3=right, ...)
    /avoid_active        (Bool)  — suspend mission during obstacle avoidance
  Publish:
    /cmd_vel             (Twist) — final velocity

Parameters:
  behaviors_file  (str)  — path to sign_behaviors.yaml
  vote_window     (int)  — sliding window for sign voting (default 10)
  vote_threshold  (float)— fraction needed to activate (default 0.5)
"""

import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, UInt8

STATE_NORMAL  = "NORMAL"
STATE_TURNING = "TURNING"


class LaneMissionNode(Node):
    def __init__(self):
        super().__init__("lane_mission")

        self.declare_parameter("behaviors_file", "")
        self.declare_parameter("vote_window",    10)
        self.declare_parameter("vote_threshold", 0.5)
        self.declare_parameter("trigger_distance_m", 0.1)  # активировать когда знак ближе N метров, -1 = игнорировать

        behaviors_file = self.get_parameter("behaviors_file").value
        self._vote_win = self.get_parameter("vote_window").value
        self._vote_thr = self.get_parameter("vote_threshold").value
        self._trigger_dist = self.get_parameter("trigger_distance_m").value

        self._behaviors = self._load_behaviors(behaviors_file)

        self._state         = STATE_NORMAL
        self._steps: list   = []
        self._step_idx      = 0
        self._step_end      = 0.0
        self._cooldowns: dict = {}
        self._sign_buf: list  = []
        self._avoid_active    = False
        self._last_lane_cmd   = Twist()
        self._sign_distance   = -1.0  # последнее значение с /detect/sign_distance

        self.create_subscription(Twist,   "/control/cmd_vel",        self._cb_cmd_vel, 1)
        self.create_subscription(UInt8,   "/detect/traffic_sign",    self._cb_sign,    10)
        self.create_subscription(Bool,    "/avoid_active",           self._cb_avoid,   10)
        self.create_subscription(Float32, "/detect/sign_distance",   self._cb_dist,    10)

        self._pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"lane_mission started. Loaded {len(self._behaviors)} sign behaviors: "
            + ", ".join(f"sign={k}({v['name']})" for k, v in self._behaviors.items())
        )

    # ------------------------------------------------------------------
    def _load_behaviors(self, path: str) -> dict:
        if not path:
            self.get_logger().warn("behaviors_file not set, using empty behaviors")
            return {}
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            behaviors = {}
            for sign_id_str, cfg in data.get("behaviors", {}).items():
                behaviors[int(sign_id_str)] = cfg
            self.get_logger().info(f"Loaded behaviors from {path}")
            return behaviors
        except Exception as e:
            self.get_logger().error(f"Failed to load {path}: {e}")
            return {}

    # ------------------------------------------------------------------
    def _cb_avoid(self, msg: Bool):
        self._avoid_active = msg.data

    def _cb_dist(self, msg: Float32):
        self._sign_distance = float(msg.data)

    def _cb_cmd_vel(self, msg: Twist):
        self._last_lane_cmd = msg
        if self._state == STATE_NORMAL and not self._avoid_active:
            self._pub.publish(msg)

    def _cb_sign(self, msg: UInt8):
        sign = int(msg.data)
        if sign not in self._behaviors:
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
            return
        cooldown = self._behaviors[sign].get("cooldown_sec", 15.0)
        if now - self._cooldowns.get(sign, 0.0) < cooldown:
            return
        # Проверка расстояния — если trigger_distance_m > 0 и знак ещё далеко, не активировать
        if self._trigger_dist > 0:
            dist = self._sign_distance
            if dist < 0 or dist > self._trigger_dist:
                return

        behavior = self._behaviors[sign]
        self._steps    = behavior.get("steps", [])
        self._step_idx = 0
        if not self._steps:
            return

        self._state = STATE_TURNING
        self._cooldowns[sign] = now
        self._sign_buf.clear()
        self._step_end = now + self._steps[0]["duration_sec"]

        self.get_logger().info(
            f"Sign {sign} ({behavior['name']}) → executing {len(self._steps)} step(s)"
        )

    # ------------------------------------------------------------------
    def _tick(self):
        now = time.time()

        if self._state == STATE_TURNING:
            if self._avoid_active:
                return

            while now > self._step_end:
                self._step_idx += 1
                if self._step_idx >= len(self._steps):
                    self._state = STATE_NORMAL
                    self.get_logger().info("Trajectory complete → NORMAL (lane following)")
                    return
                self._step_end = now + self._steps[self._step_idx]["duration_sec"]

            step = self._steps[self._step_idx]
            t = Twist()
            t.linear.x  = float(step["linear_x"])
            t.angular.z = float(step["angular_z"])
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
