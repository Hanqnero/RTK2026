"""
ROS2 нода: мост между Arduino и ROS2.

Отвечает за:
  - открытие serial порта и handshake с Arduino (через SerialTransport)
  - периодическое чтение телеметрии и публикацию /encoder_report
  - подписку на /cmd_vel и отправку линейной/угловой скорости на Arduino
  - корректную остановку моторов при завершении ноды

Локальная кинематика колёс теперь живёт на Arduino:
Pi отправляет только линейную и угловую скорость робота.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rtk2026_driver.protocol import InvalidChecksumError, pack_command, parse_telemetry
from rtk2026_driver.transport import HandshakeTimeoutError, SerialTransport
from rtk2026_interfaces.msg import EncoderReport


class ArduinoBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__("arduino_bridge")

        # --- параметры из arduino_bridge.yaml ---
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("handshake_timeout_sec", 5.0)
        self.declare_parameter("handshake_poll_interval_sec", 0.01)
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("telemetry_poll_period_sec", 0.01)
        self.declare_parameter("command_send_interval_sec", 0.02)
        self.declare_parameter("drop_stale_cmd_after_sec", 0.30)
        self.declare_parameter("left_encoder_inverted", False)
        self.declare_parameter("right_encoder_inverted", False)

        # legacy-параметры оставлены declared, чтобы старые yaml не ломали старт
        self.declare_parameter("wheel_velocity_command_topic", "wheel_velocity_command")
        self.declare_parameter("left_wheel_inverted", False)
        self.declare_parameter("right_wheel_inverted", False)

        port = self.get_parameter("serial_port").value
        baudrate = self.get_parameter("baudrate").value
        hs_timeout = self.get_parameter("handshake_timeout_sec").value
        hs_poll = self.get_parameter("handshake_poll_interval_sec").value
        enc_topic = self.get_parameter("encoder_report_topic").value
        cmd_topic = self.get_parameter("cmd_vel_topic").value
        telemetry_poll_period = float(self.get_parameter("telemetry_poll_period_sec").value)
        command_send_interval = float(self.get_parameter("command_send_interval_sec").value)
        self._drop_stale_cmd_after_sec = float(
            self.get_parameter("drop_stale_cmd_after_sec").value
        )

        # инверсия encoder counts через yaml — финальный runtime override
        self._left_enc_sign = -1 if self.get_parameter("left_encoder_inverted").value else 1
        self._right_enc_sign = -1 if self.get_parameter("right_encoder_inverted").value else 1

        self._transport = SerialTransport(
            port=port,
            baudrate=baudrate,
            handshake_timeout_sec=hs_timeout,
            handshake_poll_interval_sec=hs_poll,
        )
        self._rx_buf = bytearray()

        self._last_linear_mps = 0.0
        self._last_angular_rps = 0.0
        self._last_cmd_time_ns: int | None = None

        self._encoder_pub = self.create_publisher(EncoderReport, enc_topic, 10)
        self._cmd_sub = self.create_subscription(Twist, cmd_topic, self._on_cmd_vel, 10)

        self._read_timer = self.create_timer(telemetry_poll_period, self._read_telemetry)
        self._send_timer = self.create_timer(command_send_interval, self._send_command)

        self._open_transport(port)

        self.get_logger().info(
            "Arduino bridge started: "
            f"cmd_vel_topic={cmd_topic}, send_period={command_send_interval:.3f}s, "
            f"telemetry_poll={telemetry_poll_period:.3f}s"
        )

    def _open_transport(self, port: str) -> None:
        """Открыть serial порт и выполнить handshake с Arduino."""
        try:
            first_frame = self._transport.open()
            self.get_logger().info(
                f"Arduino handshake successful on {port} "
                f"(left_count={first_frame.left_count}, "
                f"right_count={first_frame.right_count})"
            )
        except HandshakeTimeoutError as e:
            self.get_logger().fatal(f"Handshake failed: {e}")
            raise SystemExit(1)

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Получить последнюю команду движения робота в базовой системе координат."""
        self._last_linear_mps = float(msg.linear.x)
        self._last_angular_rps = float(msg.angular.z)
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds

    def _current_command(self) -> tuple[float, float]:
        """
        Вернуть актуальную команду для отправки.

        Если cmd_vel давно не обновлялся, отправляем нули как dead-man.
        """
        if self._last_cmd_time_ns is None:
            return 0.0, 0.0

        age_sec = (self.get_clock().now().nanoseconds - self._last_cmd_time_ns) / 1e9
        if age_sec > self._drop_stale_cmd_after_sec:
            return 0.0, 0.0

        return self._last_linear_mps, self._last_angular_rps

    def _send_command(self) -> None:
        """Периодически отправлять на Arduino последнюю актуальную команду."""
        linear_mps, angular_rps = self._current_command()
        self._transport.write(pack_command(linear_mps, angular_rps))

    def _read_telemetry(self) -> None:
        """
        Таймер: читаем байты из serial буфера и публикуем EncoderReport.

        Извлекаем все полные фреймы за один вызов — буфер может накопить
        несколько фреймов если таймер сработал позже обычного.
        """
        raw = self._transport.read(256)
        if not raw:
            return

        self._rx_buf.extend(raw)

        consecutive_errors = 0
        max_consecutive_errors = 10

        while True:
            try:
                frame = parse_telemetry(self._rx_buf)
                consecutive_errors = 0
            except InvalidChecksumError as e:
                self.get_logger().warn(f"Telemetry checksum error: {e}")
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors:
                    self.get_logger().error(
                        f"Too many consecutive checksum errors ({consecutive_errors}), clearing buffer"
                    )
                    self._rx_buf.clear()
                    break
                continue

            if frame is None:
                break

            msg = EncoderReport()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.left_count = self._left_enc_sign * frame.left_count
            msg.right_count = self._right_enc_sign * frame.right_count
            self._encoder_pub.publish(msg)

    def _stop_motors(self) -> None:
        """Отправить нулевую команду движения — остановить оба борта."""
        self._transport.write(pack_command(0.0, 0.0))

    def destroy_node(self) -> None:
        """Корректное завершение: останавливаем моторы и закрываем порт."""
        self._stop_motors()
        self._transport.close()
        super().destroy_node()


def main(args=None) -> None:
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
