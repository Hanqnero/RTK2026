# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import struct
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from rtk2026_interfaces.msg import EncoderReport, WheelVelocityCommand


CONTROL_FORMAT = "<ffB"
TELEMETRY_FORMAT = "<fffiihhff"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)


def pack_control(linear_mps: float, angular_rps: float) -> bytes:
    return struct.pack(CONTROL_FORMAT, float(linear_mps), float(angular_rps), 1)


def wheel_ticks_to_body_command(
    left_tps: int,
    right_tps: int,
    ticks_per_meter: float,
    wheel_separation: float,
) -> tuple[float, float]:
    left_mps = float(left_tps) / ticks_per_meter
    right_mps = float(right_tps) / ticks_per_meter
    return (left_mps + right_mps) * 0.5, -((right_mps - left_mps) / wheel_separation)


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("read_timeout_sec", 0.5)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("wheel_velocity_command_topic", "wheel_velocity_command")
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("min_send_interval_sec", 0.05)
        self.declare_parameter("drop_stale_cmd_after_sec", 0.30)
        self.declare_parameter("left_motor_inverted", False)
        self.declare_parameter("right_motor_inverted", False)
        self.declare_parameter("left_encoder_inverted", False)
        self.declare_parameter("right_encoder_inverted", False)
        self.declare_parameter("ticks_per_meter", 3000.0)
        self.declare_parameter("wheel_separation", 0.24)
        self.declare_parameter("control_period_ms", 100)

        baud_rate = int(self.get_parameter("baud_rate").value)
        legacy_baudrate = int(self.get_parameter("baudrate").value)
        self._baud = legacy_baudrate if baud_rate == 115200 and legacy_baudrate != 115200 else baud_rate
        self._port_path = self.get_parameter("serial_port").value
        self._read_timeout = float(self.get_parameter("read_timeout_sec").value)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._min_send_interval = float(self.get_parameter("min_send_interval_sec").value)
        self._drop_stale_cmd_after_sec = float(self.get_parameter("drop_stale_cmd_after_sec").value)
        self._left_motor_sign = -1 if self.get_parameter("left_motor_inverted").value else 1
        self._right_motor_sign = -1 if self.get_parameter("right_motor_inverted").value else 1
        self._left_encoder_sign = -1 if self.get_parameter("left_encoder_inverted").value else 1
        self._right_encoder_sign = -1 if self.get_parameter("right_encoder_inverted").value else 1
        self._ticks_per_meter = float(self.get_parameter("ticks_per_meter").value)
        self._wheel_separation = float(self.get_parameter("wheel_separation").value)
        self._control_period_sec = float(self.get_parameter("control_period_ms").value) / 1000.0

        if self._ticks_per_meter <= 0.0:
            raise ValueError("ticks_per_meter must be > 0")
        if self._wheel_separation <= 0.0:
            raise ValueError("wheel_separation must be > 0")
        if self._control_period_sec <= 0.0:
            raise ValueError("control_period_ms must be > 0")

        self._ser = None
        self._rx = bytearray()
        self._left_count = 0
        self._right_count = 0
        self._pending_payload = pack_control(0.0, 0.0)
        self._last_send_time = 0.0
        self._last_cmd_time_ns = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._encoder_pub = self.create_publisher(
            EncoderReport,
            self.get_parameter("encoder_report_topic").value,
            qos,
        )
        self.create_subscription(Twist, self.get_parameter("cmd_vel_topic").value, self._on_cmd_vel, 10)
        self.create_subscription(
            WheelVelocityCommand,
            self.get_parameter("wheel_velocity_command_topic").value,
            self._on_wheel_velocity,
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
            time.sleep(3.0)
            self._ser.reset_input_buffer()
            self._rx.clear()
            self.get_logger().info("Opened serial %s at %d" % (self._port_path, self._baud))
        except Exception as e:
            self.get_logger().warn("Could not open serial %s: %s" % (self._port_path, e))
            self._ser = None

    def _on_cmd_vel(self, msg: Twist):
        self._pending_payload = pack_control(msg.linear.x, msg.angular.z)
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds

    def _on_wheel_velocity(self, msg: WheelVelocityCommand):
        left_tps = self._left_motor_sign * int(msg.left_tps)
        right_tps = self._right_motor_sign * int(msg.right_tps)
        linear_mps, angular_rps = wheel_ticks_to_body_command(
            left_tps,
            right_tps,
            self._ticks_per_meter,
            self._wheel_separation,
        )
        self._pending_payload = pack_control(linear_mps, angular_rps)
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds

    def _timer_cb(self):
        if self._ser is None or not self._ser.is_open:
            self._open_serial()
            return

        try:
            self._read_telemetry()
            self._write_command()
        except Exception as e:
            self.get_logger().warn("Serial error: %s" % e)
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _read_telemetry(self):
        raw = self._ser.read(256)
        if raw:
            self._rx.extend(raw)

        while len(self._rx) >= TELEMETRY_SIZE:
            packet = bytes(self._rx[:TELEMETRY_SIZE])
            del self._rx[:TELEMETRY_SIZE]
            (
                _odom_x_m,
                _odom_y_m,
                _odom_heading_rad,
                left_delta,
                right_delta,
                _left_pwm,
                _right_pwm,
                _current_linear_mps,
                _current_angular_rps,
            ) = struct.unpack(TELEMETRY_FORMAT, packet)
            self._left_count += int(left_delta)
            self._right_count += int(right_delta)
            self._publish_encoder_report(left_delta, right_delta)

    def _publish_encoder_report(self, left_delta: int, right_delta: int):
        report = EncoderReport()
        report.header = Header()
        report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = "base_link"
        report.left_count = self._left_encoder_sign * self._left_count
        report.right_count = self._right_encoder_sign * self._right_count
        if hasattr(report, "left_speed"):
            report.left_speed = self._left_encoder_sign * int(round(left_delta / self._control_period_sec))
        if hasattr(report, "right_speed"):
            report.right_speed = self._right_encoder_sign * int(round(right_delta / self._control_period_sec))
        self._encoder_pub.publish(report)

    def _write_command(self):
        now = time.monotonic()
        if now - self._last_send_time < self._min_send_interval:
            return

        if self._last_cmd_time_ns is not None:
            age = (self.get_clock().now().nanoseconds - self._last_cmd_time_ns) / 1e9
            if age > self._drop_stale_cmd_after_sec:
                self._pending_payload = pack_control(0.0, 0.0)

        self._ser.write(self._pending_payload)
        self._last_send_time = now

    def destroy_node(self):
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.write(pack_control(0.0, 0.0))
            except Exception:
                pass
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
