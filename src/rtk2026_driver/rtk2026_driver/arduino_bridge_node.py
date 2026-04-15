#!/usr/bin/env python3

import struct
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from serial import Serial, SerialException

from rtk2026_driver.msg import TelemetryPacket


class ArduinoBridgeNode(Node):
    """
    ROS2 <-> Arduino serial bridge.

    - Subscribes:  /cmd_vel (geometry_msgs/Twist)
    - Publishes:   /arduino/telemetry (rtk2026_driver/TelemetryPacket)
    - Sends control packet every 100 ms
    - Reads telemetry packet every 100 ms (best-effort with buffered parser)

    Protocol layout is mirrored from arduino/include/control_protocol.h:

      ControlPacket (8 bytes):
        float target_linear_mps
        float target_angular_rps

      TelemetryPacket (48 bytes):
        float   target_linear_mps
        float   target_angular_rps
        float   current_linear_mps
        float   current_angular_rps
        float   target_left_wheel_mps
        float   target_right_wheel_mps
        float   current_left_wheel_mps
        float   current_right_wheel_mps
        int16   left_pwm
        int16   right_pwm
        int32   left_count
        int32   right_count
    """

    CONTROL_PERIOD_S = 0.1  # 100 ms
    TELEMETRY_PERIOD_S = 0.1  # 100 ms

    # Little-endian, tightly packed
    CONTROL_STRUCT = struct.Struct("<ff")
    TELEMETRY_STRUCT = struct.Struct("<ffffffffhhii")

    def __init__(self) -> None:
        super().__init__("arduino_bridge_node")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("telemetry_topic", "/arduino/telemetry")
        self.declare_parameter("serial_timeout_s", 0.02)  # non-blocking-ish reads
        self.declare_parameter("reconnect_period_s", 1.0)
        self.declare_parameter("drop_stale_cmd_after_s", 0.5)

        self._port = self.get_parameter("port").get_parameter_value().string_value
        self._baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )
        self._cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self._telemetry_topic = (
            self.get_parameter("telemetry_topic").get_parameter_value().string_value
        )
        self._serial_timeout_s = (
            self.get_parameter("serial_timeout_s").get_parameter_value().double_value
        )
        self._reconnect_period_s = (
            self.get_parameter("reconnect_period_s").get_parameter_value().double_value
        )
        self._drop_stale_cmd_after_s = (
            self.get_parameter("drop_stale_cmd_after_s")
            .get_parameter_value()
            .double_value
        )

        # ROS interfaces
        self._telemetry_pub = self.create_publisher(
            TelemetryPacket, self._telemetry_topic, 20
        )
        self._cmd_sub = self.create_subscription(
            Twist, self._cmd_vel_topic, self._on_cmd_vel, 20
        )

        # Shared command state
        self._lock = threading.Lock()
        self._target_linear_mps = 0.0
        self._target_angular_rps = 0.0
        self._last_cmd_time = self.get_clock().now()

        # Serial
        self._serial: Optional[Serial] = None
        self._rx_buffer = bytearray()

        # Timers
        self._tx_timer = self.create_timer(
            self.CONTROL_PERIOD_S, self._send_control_packet
        )
        self._rx_timer = self.create_timer(
            self.TELEMETRY_PERIOD_S, self._read_and_publish_telemetry
        )
        self._reconnect_timer = self.create_timer(
            self._reconnect_period_s, self._ensure_serial_connected
        )

        self._open_serial()

        self.get_logger().info(
            f"Arduino bridge started | port={self._port}, baudrate={self._baudrate}, "
            f"control_period={self.CONTROL_PERIOD_S}s, telemetry_period={self.TELEMETRY_PERIOD_S}s"
        )

    def _open_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            return

        try:
            self._serial = Serial(
                port=self._port,
                baudrate=int(self._baudrate),
                timeout=float(self._serial_timeout_s),
                write_timeout=float(self._serial_timeout_s),
            )
            # Let Arduino settle after reset on serial open
            time.sleep(1.5)
            self._rx_buffer.clear()
            self.get_logger().info(f"Connected to Arduino serial: {self._port}")
        except SerialException as exc:
            self._serial = None
            self.get_logger().warn(f"Serial open failed ({self._port}): {exc}")

    def _close_serial(self) -> None:
        if self._serial is None:
            return
        try:
            if self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        finally:
            self._serial = None

    def _ensure_serial_connected(self) -> None:
        if self._serial is None or not self._serial.is_open:
            self._open_serial()

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._target_linear_mps = float(msg.linear.x)
            self._target_angular_rps = float(msg.angular.z)
            self._last_cmd_time = self.get_clock().now()

    def _get_command_for_tx(self) -> tuple[float, float]:
        with self._lock:
            linear = self._target_linear_mps
            angular = self._target_angular_rps
            last_cmd_time = self._last_cmd_time

        age_s = (self.get_clock().now() - last_cmd_time).nanoseconds * 1e-9
        if age_s > self._drop_stale_cmd_after_s:
            return 0.0, 0.0
        return linear, angular

    def _send_control_packet(self) -> None:
        if self._serial is None or not self._serial.is_open:
            return

        linear, angular = self._get_command_for_tx()
        payload = self.CONTROL_STRUCT.pack(linear, angular)

        try:
            self._serial.write(payload)
        except (SerialException, OSError) as exc:
            self.get_logger().warn(f"Serial write failed: {exc}")
            self._close_serial()

    def _read_and_publish_telemetry(self) -> None:
        if self._serial is None or not self._serial.is_open:
            return

        packet_size = self.TELEMETRY_STRUCT.size

        try:
            available = self._serial.in_waiting
            if available > 0:
                data = self._serial.read(available)
                if data:
                    self._rx_buffer.extend(data)

            # Parse any complete packets in the buffer.
            # With no explicit framing/checksum in protocol header, we consume
            # fixed-size chunks best-effort.
            while len(self._rx_buffer) >= packet_size:
                raw = bytes(self._rx_buffer[:packet_size])
                del self._rx_buffer[:packet_size]

                fields = self.TELEMETRY_STRUCT.unpack(raw)

                msg = TelemetryPacket()
                msg.target_linear_mps = float(fields[0])
                msg.target_angular_rps = float(fields[1])
                msg.current_linear_mps = float(fields[2])
                msg.current_angular_rps = float(fields[3])
                msg.target_left_wheel_mps = float(fields[4])
                msg.target_right_wheel_mps = float(fields[5])
                msg.current_left_wheel_mps = float(fields[6])
                msg.current_right_wheel_mps = float(fields[7])
                msg.left_pwm = int(fields[8])
                msg.right_pwm = int(fields[9])
                msg.left_count = int(fields[10])
                msg.right_count = int(fields[11])

                self._telemetry_pub.publish(msg)

            # Prevent unbounded growth in case of line noise / desync
            max_buffer = packet_size * 20
            if len(self._rx_buffer) > max_buffer:
                # Keep only the most recent bytes
                self._rx_buffer = self._rx_buffer[-packet_size:]

        except (SerialException, OSError) as exc:
            self.get_logger().warn(f"Serial read failed: {exc}")
            self._close_serial()

    def destroy_node(self) -> bool:
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
