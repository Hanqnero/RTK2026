# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rtk2026_interfaces.msg import EncoderReport, MotorCommand
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion


class BaseControllerNode(Node):
    def __init__(self):
        super().__init__("base_controller")

        self.declare_parameter("wheel_separation", 0.3)
        self.declare_parameter("ticks_per_meter", 1000.0)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("max_pwm", 127)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("motor_command_topic", "motor_command")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("tf_publish_rate", 20.0)
        self.declare_parameter("publish_tf", True)

        self._wheel_sep = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self._ticks_per_m = self.get_parameter("ticks_per_meter").get_parameter_value().double_value
        self._max_linear = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self._max_angular = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self._max_pwm = self.get_parameter("max_pwm").get_parameter_value().integer_value
        self._cmd_timeout = self.get_parameter("cmd_vel_timeout_sec").get_parameter_value().double_value
        self._odom_frame = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame_id").get_parameter_value().string_value
        self._tf_rate = self.get_parameter("tf_publish_rate").get_parameter_value().double_value
        pv = self.get_parameter("publish_tf").get_parameter_value()
        if pv.type == ParameterType.PARAMETER_BOOL:
            self._publish_tf = pv.bool_value
        else:
            self._publish_tf = pv.string_value.lower() == "true"

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_left = 0
        self._last_right = 0
        self._last_encoder_time = None
        self._last_cmd_vel_time = None
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0

        self._motor_pub = self.create_publisher(
            MotorCommand,
            self.get_parameter("motor_command_topic").get_parameter_value().string_value,
            10,
        )
        self._odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            10,
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        self._cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            self._cmd_vel_cb,
            10,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._encoder_sub = self.create_subscription(
            EncoderReport,
            self.get_parameter("encoder_report_topic").get_parameter_value().string_value,
            self._encoder_cb,
            qos_sensor,
        )

        self._cmd_timer = self.create_timer(0.05, self._cmd_timer_cb)
        if self._publish_tf:
            self._tf_timer = self.create_timer(1.0 / self._tf_rate, self._tf_timer_cb)
        else:
            self._tf_timer = None

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_vel_time = self.get_clock().now()
        self._last_cmd_linear = msg.linear.x
        self._last_cmd_angular = msg.angular.z

    def _cmd_timer_cb(self):
        now = self.get_clock().now()
        if self._last_cmd_vel_time is not None:
            elapsed = (now - self._last_cmd_vel_time).nanoseconds / 1e9
            if elapsed > self._cmd_timeout:
                self._last_cmd_linear = 0.0
                self._last_cmd_angular = 0.0
        v = max(-self._max_linear, min(self._max_linear, self._last_cmd_linear))
        omega = max(-self._max_angular, min(self._max_angular, self._last_cmd_angular))
        L = self._wheel_sep
        v_l = v - omega * L / 2.0
        v_r = v + omega * L / 2.0
        if self._max_linear > 0:
            pwm_l = int((v_l / self._max_linear) * self._max_pwm)
            pwm_r = int((v_r / self._max_linear) * self._max_pwm)
        else:
            pwm_l = 0
            pwm_r = 0
        pwm_l = max(-self._max_pwm, min(self._max_pwm, pwm_l))
        pwm_r = max(-self._max_pwm, min(self._max_pwm, pwm_r))
        self._motor_pub.publish(MotorCommand(left_pwm=pwm_l, right_pwm=pwm_r))

    def _encoder_cb(self, msg: EncoderReport):
        now = self.get_clock().now()
        left = msg.left_count
        right = msg.right_count
        if self._last_encoder_time is not None:
            dt = (now - self._last_encoder_time).nanoseconds / 1e9
            if dt <= 0:
                return
            meters_per_tick = 1.0 / self._ticks_per_m
            d_left = (left - self._last_left) * meters_per_tick
            d_right = (right - self._last_right) * meters_per_tick
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self._wheel_sep
            self._theta += d_theta
            self._x += d_center * math.cos(self._theta)
            self._y += d_center * math.sin(self._theta)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self._odom_frame
            odom.child_frame_id = self._base_frame
            odom.pose.pose.position.x = self._x
            odom.pose.pose.position.y = self._y
            odom.pose.pose.position.z = 0.0
            q = self._euler_to_quat(0.0, 0.0, self._theta)
            odom.pose.pose.orientation = q
            speed_period_hz = 10.0
            v_left = msg.left_speed * meters_per_tick * speed_period_hz
            v_right = msg.right_speed * meters_per_tick * speed_period_hz
            odom.twist.twist.linear.x = (v_left + v_right) * 0.5
            odom.twist.twist.angular.z = (v_right - v_left) / self._wheel_sep
            self._odom_pub.publish(odom)
        self._last_left = left
        self._last_right = right
        self._last_encoder_time = now

    def _euler_to_quat(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def _tf_timer_cb(self):
        if not self._publish_tf or self._last_encoder_time is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation = self._euler_to_quat(0.0, 0.0, self._theta)
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
