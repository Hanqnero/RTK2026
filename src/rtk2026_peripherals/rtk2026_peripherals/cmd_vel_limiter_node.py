#!/usr/bin/env python3
# Ограничивает linear.x и angular.z. Для Autorace: при avoid_active публикует
# /avoid_control (ограниченный), иначе — /cmd_vel_raw (ограниченный) в /cmd_vel.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelLimiter(Node):
    def __init__(self):
        super().__init__("cmd_vel_limiter")
        self.declare_parameter("max_linear", 0.25)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("avoid_control_topic", "/avoid_control")
        self.declare_parameter("avoid_active_topic", "/avoid_active")
        self.declare_parameter("use_avoid_merge", True)

        max_linear = self.get_parameter("max_linear").value
        max_angular = self.get_parameter("max_angular").value
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        avoid_ctrl = self.get_parameter("avoid_control_topic").value
        avoid_active = self.get_parameter("avoid_active_topic").value
        use_merge = self.get_parameter("use_avoid_merge").value

        self._max_linear = max_linear
        self._max_angular = max_angular
        self._use_avoid_merge = use_merge
        self._avoid_active = False
        self._last_avoid_control = Twist()

        self._sub_lane = self.create_subscription(Twist, in_topic, self._cb_lane, 10)
        self._pub = self.create_publisher(Twist, out_topic, 10)

        if use_merge:
            self._sub_avoid = self.create_subscription(
                Twist, avoid_ctrl, self._cb_avoid_control, 10
            )
            self._sub_active = self.create_subscription(
                Bool, avoid_active, self._cb_avoid_active, 10
            )

    def _clip(self, msg: Twist) -> Twist:
        out = Twist()
        out.linear.x = max(-self._max_linear, min(self._max_linear, msg.linear.x))
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = max(-self._max_angular, min(self._max_angular, msg.angular.z))
        return out

    def _cb_lane(self, msg: Twist):
        if self._use_avoid_merge and self._avoid_active:
            self._pub.publish(self._clip(self._last_avoid_control))
            return
        self._pub.publish(self._clip(msg))

    def _cb_avoid_control(self, msg: Twist):
        self._last_avoid_control = msg
        if self._use_avoid_merge and self._avoid_active:
            self._pub.publish(self._clip(msg))

    def _cb_avoid_active(self, msg: Bool):
        self._avoid_active = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLimiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
