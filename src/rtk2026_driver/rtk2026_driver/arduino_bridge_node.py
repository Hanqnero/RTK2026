# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from rtk2026_interfaces.msg import EncoderReport, MotorCommand

try:
    import serial
except ImportError:
    serial = None


def clamp_pwm(value: int, low: int = -128, high: int = 127) -> int:
    return max(low, min(high, value))


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("read_timeout_sec", 0.5)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("motor_command_topic", "motor_command")
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("min_motor_send_interval_sec", 0.05)

        self._port_path = self.get_parameter("serial_port").get_parameter_value().string_value
        self._baud = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self._read_timeout = self.get_parameter("read_timeout_sec").get_parameter_value().double_value
        self._publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._motor_topic = self.get_parameter("motor_command_topic").get_parameter_value().string_value
        self._encoder_topic = self.get_parameter("encoder_report_topic").get_parameter_value().string_value
        self._min_send_interval = self.get_parameter("min_motor_send_interval_sec").get_parameter_value().double_value

        self._ser = None
        self._last_motor_send_time = 0.0
        self._pending_left = 0
        self._pending_right = 0
        self._has_pending = False

        if serial is None:
            self.get_logger().error("pyserial not installed")
            return

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self._encoder_pub = self.create_publisher(EncoderReport, self._encoder_topic, qos)
        self._motor_sub = self.create_subscription(
            MotorCommand,
            self._motor_topic,
            self._motor_cb,
            10,
        )

        self._timer = self.create_timer(1.0 / self._publish_rate, self._timer_cb)
        self._open_serial()

    def _open_serial(self):
        try:
            self._ser = serial.Serial(
                port=self._port_path,
                baudrate=self._baud,
                timeout=self._read_timeout,
                write_timeout=0.1,
            )
            self.get_logger().info("Opened serial %s at %d" % (self._port_path, self._baud))
        except Exception as e:
            self.get_logger().warn("Could not open serial %s: %s" % (self._port_path, e))
            self._ser = None

    def _motor_cb(self, msg: MotorCommand):
        self._pending_left = clamp_pwm(msg.left_pwm)
        self._pending_right = clamp_pwm(msg.right_pwm)
        self._has_pending = True

    def _timer_cb(self):
        if self._ser is None or not self._ser.is_open:
            return
        try:
            if self._ser.in_waiting >= 32:
                raw = self._ser.read(32)
                if len(raw) == 32:
                    left_speed = struct.unpack("<i", raw[0:4])[0]
                    left_cnt = struct.unpack("<i", raw[4:8])[0]
                    right_speed = struct.unpack("<i", raw[8:12])[0]
                    right_cnt = struct.unpack("<i", raw[12:16])[0]
                    report = EncoderReport()
                    report.header = Header()
                    report.header.stamp = self.get_clock().now().to_msg()
                    report.header.frame_id = "base_link"
                    report.left_count = int(left_cnt)
                    report.right_count = int(right_cnt)
                    report.left_speed = left_speed
                    report.right_speed = right_speed
                    self._encoder_pub.publish(report)
        except Exception as e:
            self.get_logger().warn("Serial read error: %s" % e)
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self._open_serial()
            return

        now = time.monotonic()
        if self._has_pending and (now - self._last_motor_send_time) >= self._min_send_interval:
            try:
                b0 = self._pending_left & 0xFF
                b1 = self._pending_right & 0xFF
                self._ser.write(bytes([b0, b1]))
                self._last_motor_send_time = now
                self._has_pending = False
            except Exception as e:
                self.get_logger().warn("Serial write error: %s" % e)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    if serial is not None:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
