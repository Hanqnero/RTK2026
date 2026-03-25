# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from rtk2026_interfaces.msg import EncoderReport, MotorCommand

try:
    import serial
except ImportError:
    serial = None


RX_PACKET_SIZE = 16  # 4 x int32: left_speed, left_cnt, right_speed, right_cnt


def clamp_pwm(value: int, low: int = -255, high: int = 255) -> int:
    return max(low, min(high, value))


def pwm_to_dir_bytes(pwm: int) -> tuple[int, int]:
    pwm = clamp_pwm(pwm)
    if pwm >= 0:
        return pwm & 0xFF, 0
    return 0, (-pwm) & 0xFF


class ArduinoBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("arduino_bridge")

        # Raspberry Pi GPIO UART alias (pins 8/10)
        self.declare_parameter("serial_port", "/dev/serial0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("read_timeout_sec", 0.05)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("motor_command_topic", "motor_command")
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("min_motor_send_interval_sec", 0.02)

        self._port_path = str(self.get_parameter("serial_port").value)
        self._baud = int(self.get_parameter("baud_rate").value)
        self._read_timeout = float(self.get_parameter("read_timeout_sec").value)
        self._publish_rate = float(self.get_parameter("publish_rate").value)
        self._motor_topic = str(self.get_parameter("motor_command_topic").value)
        self._encoder_topic = str(self.get_parameter("encoder_report_topic").value)
        self._min_send_interval = float(
            self.get_parameter("min_motor_send_interval_sec").value
        )

        self._ser = None
        self._rx_buf = bytearray()
        self._last_motor_send_time = 0.0
        self._pending_left = 0
        self._pending_right = 0
        self._has_pending = False

        if serial is None:
            self.get_logger().error("pyserial is not installed")
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._encoder_pub = self.create_publisher(
            EncoderReport, self._encoder_topic, qos
        )
        self._motor_sub = self.create_subscription(
            MotorCommand,
            self._motor_topic,
            self._motor_cb,
            10,
        )

        period = max(0.001, 1.0 / max(self._publish_rate, 1.0))
        self._timer = self.create_timer(period, self._timer_cb)

        self._open_serial()

    def _open_serial(self) -> None:
        try:
            self._ser = serial.Serial(
                port=self._port_path,
                baudrate=self._baud,
                timeout=self._read_timeout,
                write_timeout=0.1,
            )
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._rx_buf.clear()
            self.get_logger().info(f"Opened UART {self._port_path} at {self._baud} bps")
        except Exception as e:
            self.get_logger().warning(f"Could not open UART {self._port_path}: {e}")
            self._ser = None

    def _reconnect_serial(self, reason: str) -> None:
        self.get_logger().warning(reason)
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._open_serial()

    def _motor_cb(self, msg: MotorCommand) -> None:
        self._pending_left = clamp_pwm(msg.left_pwm)
        self._pending_right = clamp_pwm(msg.right_pwm)
        self._has_pending = True

    def _read_reports(self) -> None:
        if self._ser is None or not self._ser.is_open:
            return

        try:
            waiting = self._ser.in_waiting
            if waiting > 0:
                self._rx_buf.extend(self._ser.read(waiting))

            while len(self._rx_buf) >= RX_PACKET_SIZE:
                raw = bytes(self._rx_buf[:RX_PACKET_SIZE])
                del self._rx_buf[:RX_PACKET_SIZE]

                left_speed, left_cnt, right_speed, right_cnt = struct.unpack(
                    "<iiii", raw
                )

                report = EncoderReport()
                report.header = Header()
                report.header.stamp = self.get_clock().now().to_msg()
                report.header.frame_id = "base_link"
                report.left_count = int(left_cnt)
                report.right_count = int(right_cnt)
                report.left_speed = int(left_speed)
                report.right_speed = int(right_speed)
                self._encoder_pub.publish(report)
        except Exception as e:
            self._reconnect_serial(f"UART read error on {self._port_path}: {e}")

    def _write_motor_command(self) -> None:
        if self._ser is None or not self._ser.is_open or not self._has_pending:
            return

        now = time.monotonic()
        if (now - self._last_motor_send_time) < self._min_send_interval:
            return

        try:
            left_fwd, left_bwd = pwm_to_dir_bytes(self._pending_left)
            right_fwd, right_bwd = pwm_to_dir_bytes(self._pending_right)
            self._ser.write(bytes([left_fwd, left_bwd, right_fwd, right_bwd]))
            self._last_motor_send_time = now
            self._has_pending = False
        except Exception as e:
            self._reconnect_serial(f"UART write error on {self._port_path}: {e}")

    def _timer_cb(self) -> None:
        if self._ser is None or not self._ser.is_open:
            self._open_serial()
            return

        self._read_reports()
        self._write_motor_command()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    if serial is not None:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
