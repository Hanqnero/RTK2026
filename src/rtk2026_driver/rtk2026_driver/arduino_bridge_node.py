# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0

import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
try:
    from rtk2026_interfaces.msg import EncoderReport, MotorCommand, WheelVelocityCommand
    HAS_WHEEL_VELOCITY_MSG = True
except ImportError:
    from rtk2026_interfaces.msg import EncoderReport, MotorCommand
    WheelVelocityCommand = None
    HAS_WHEEL_VELOCITY_MSG = False

try:
    import serial
except ImportError:
    serial = None


TX_PACKET_SIZE = 16  # legacy: 4x int32_t (left_speed, left_cnt, right_speed, right_cnt)
TX_FRAMED_PACKET_SIZE = 19  # 2-byte header + 16-byte payload + 1-byte checksum
VELOCITY_PACKET_HEADER = b"\xA5\x5A"
TELEMETRY_PACKET_HEADER = b"\x5A\xA5"


def clamp_pwm(value: int, low: int = -255, high: int = 255) -> int:
    return max(low, min(high, value))


def clamp_i16(value: int) -> int:
    return max(-32768, min(32767, value))


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("read_timeout_sec", 0.5)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("command_mode", "pwm")
        self.declare_parameter("allow_legacy_raw_telemetry", False)
        self.declare_parameter("motor_command_topic", "motor_command")
        self.declare_parameter("wheel_velocity_command_topic", "wheel_velocity_command")
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("min_motor_send_interval_sec", 0.05)
        self.declare_parameter("left_motor_inverted", False)
        self.declare_parameter("right_motor_inverted", False)
        self.declare_parameter("left_encoder_inverted", False)
        self.declare_parameter("right_encoder_inverted", False)

        self._port_path = self.get_parameter("serial_port").get_parameter_value().string_value
        self._baud = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self._read_timeout = self.get_parameter("read_timeout_sec").get_parameter_value().double_value
        self._publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._command_mode = self.get_parameter("command_mode").get_parameter_value().string_value
        self._allow_legacy_raw = self.get_parameter("allow_legacy_raw_telemetry").get_parameter_value().bool_value
        self._motor_topic = self.get_parameter("motor_command_topic").get_parameter_value().string_value
        self._wheel_velocity_topic = self.get_parameter("wheel_velocity_command_topic").get_parameter_value().string_value
        self._encoder_topic = self.get_parameter("encoder_report_topic").get_parameter_value().string_value
        self._min_send_interval = self.get_parameter("min_motor_send_interval_sec").get_parameter_value().double_value
        self._left_motor_inv = self.get_parameter("left_motor_inverted").get_parameter_value().bool_value
        self._right_motor_inv = self.get_parameter("right_motor_inverted").get_parameter_value().bool_value
        self._left_enc_inv = self.get_parameter("left_encoder_inverted").get_parameter_value().bool_value
        self._right_enc_inv = self.get_parameter("right_encoder_inverted").get_parameter_value().bool_value

        self._ser = None
        self._last_motor_send_time = 0.0
        self._pending_payload = None
        self._has_pending = False
        self._telemetry_rx = bytearray()

        if serial is None:
            self.get_logger().error("pyserial not installed")
            return

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self._encoder_pub = self.create_publisher(EncoderReport, self._encoder_topic, qos)
        if self._command_mode == "wheel_velocity" and HAS_WHEEL_VELOCITY_MSG:
            self._command_sub = self.create_subscription(
                WheelVelocityCommand,
                self._wheel_velocity_topic,
                self._wheel_velocity_cb,
                10,
            )
            self.get_logger().info(
                "Arduino bridge command mode: wheel_velocity on topic '%s'" % self._wheel_velocity_topic
            )
        elif self._command_mode == "wheel_velocity" and not HAS_WHEEL_VELOCITY_MSG:
            self.get_logger().warn(
                "WheelVelocityCommand msg is unavailable in this workspace, falling back to pwm mode"
            )
            self._command_mode = "pwm"
            self._command_sub = self.create_subscription(
                MotorCommand,
                self._motor_topic,
                self._motor_cb,
                10,
            )
            self.get_logger().info(
                "Arduino bridge command mode: pwm on topic '%s'" % self._motor_topic
            )
        else:
            self._command_sub = self.create_subscription(
                MotorCommand,
                self._motor_topic,
                self._motor_cb,
                10,
            )
            self.get_logger().info(
                "Arduino bridge command mode: pwm on topic '%s'" % self._motor_topic
            )

        self._timer = self.create_timer(1.0 / self._publish_rate, self._timer_cb)
        self._open_serial()

    def _publish_encoder_report(self, left_speed: int, left_cnt: int, right_speed: int, right_cnt: int):
        report = EncoderReport()
        report.header = Header()
        report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = "base_link"
        report.left_count = int(-left_cnt if self._left_enc_inv else left_cnt)
        report.right_count = int(-right_cnt if self._right_enc_inv else right_cnt)
        report.left_speed = -left_speed if self._left_enc_inv else left_speed
        report.right_speed = -right_speed if self._right_enc_inv else right_speed
        self._encoder_pub.publish(report)

    def _try_read_framed_telemetry(self):
        try:
            available = self._ser.in_waiting
        except Exception:
            return False

        if available > 0:
            chunk = self._ser.read(available)
            if chunk:
                self._telemetry_rx.extend(chunk)

        parsed = False
        while len(self._telemetry_rx) >= TX_FRAMED_PACKET_SIZE:
            header_index = self._telemetry_rx.find(TELEMETRY_PACKET_HEADER)
            if header_index < 0:
                # keep only the last byte in case it is the first half of the header
                del self._telemetry_rx[:-1]
                return parsed

            if header_index > 0:
                del self._telemetry_rx[:header_index]

            if len(self._telemetry_rx) < TX_FRAMED_PACKET_SIZE:
                return parsed

            raw = bytes(self._telemetry_rx[:TX_FRAMED_PACKET_SIZE])
            checksum = sum(raw[:-1]) & 0xFF
            if checksum != raw[-1]:
                self.get_logger().warn("Telemetry checksum mismatch, dropping frame")
                del self._telemetry_rx[:2]
                parsed = True
                continue

            left_speed, left_cnt, right_speed, right_cnt = struct.unpack("<iiii", raw[2:-1])
            self._publish_encoder_report(left_speed, left_cnt, right_speed, right_cnt)
            del self._telemetry_rx[:TX_FRAMED_PACKET_SIZE]
            parsed = True

        return parsed

    def _open_serial(self):
        try:
            self._ser = serial.Serial(
                port=self._port_path,
                baudrate=self._baud,
                timeout=self._read_timeout,
                write_timeout=0.1,
            )
            # CH340 сбрасывает Arduino при открытии порта — ждём загрузки bootloader+sketch
            import time as _time
            _time.sleep(3.0)
            self._ser.reset_input_buffer()
            self._telemetry_rx.clear()
            self.get_logger().info("Opened serial %s at %d" % (self._port_path, self._baud))
        except Exception as e:
            self.get_logger().warn("Could not open serial %s: %s" % (self._port_path, e))
            self._ser = None

    def _motor_cb(self, msg: MotorCommand):
        left = msg.left_pwm if not self._left_motor_inv else -msg.left_pwm
        right = msg.right_pwm if not self._right_motor_inv else -msg.right_pwm
        left = clamp_pwm(left)
        right = clamp_pwm(right)
        self._pending_payload = bytes([
            max(left, 0) & 0xFF,
            max(-left, 0) & 0xFF,
            max(right, 0) & 0xFF,
            max(-right, 0) & 0xFF,
        ])
        self._has_pending = True

    def _wheel_velocity_cb(self, msg: WheelVelocityCommand):
        left = msg.left_ticks_per_sec if not self._left_motor_inv else -msg.left_ticks_per_sec
        right = msg.right_ticks_per_sec if not self._right_motor_inv else -msg.right_ticks_per_sec
        left_tps = clamp_i16(int(left))
        right_tps = clamp_i16(int(right))
        payload_wo_crc = VELOCITY_PACKET_HEADER + struct.pack("<hh", left_tps, right_tps)
        checksum = sum(payload_wo_crc) & 0xFF
        self._pending_payload = payload_wo_crc + bytes([checksum])
        self._has_pending = True

    def _timer_cb(self):
        if self._ser is None or not self._ser.is_open:
            self._open_serial()
            return
        try:
            framed = self._try_read_framed_telemetry()
            if (
                self._allow_legacy_raw
                and self._command_mode != "wheel_velocity"
                and not framed
                and len(self._telemetry_rx) == 0
                and self._ser.in_waiting >= TX_PACKET_SIZE
            ):
                raw = self._ser.read(TX_PACKET_SIZE)
                if len(raw) == TX_PACKET_SIZE:
                    left_speed, left_cnt, right_speed, right_cnt = struct.unpack("<iiii", raw)
                    self._publish_encoder_report(left_speed, left_cnt, right_speed, right_cnt)
        except Exception as e:
            self.get_logger().warn("Serial read error: %s" % e)
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self._open_serial()
            # не делаем return — write всё равно должен отработать если порт переоткрылся

        now = time.monotonic()
        if self._has_pending and (now - self._last_motor_send_time) >= self._min_send_interval:
            try:
                self._ser.write(self._pending_payload)
                self._last_motor_send_time = now
                self._has_pending = False
            except Exception as e:
                self.get_logger().warn("Serial write error: %s" % e)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    if serial is not None:
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
    node.destroy_node()
    rclpy.shutdown()
