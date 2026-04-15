"""
ROS2 нода: мост между Arduino и ROS2.

Отвечает за:
  - открытие serial порта и handshake с Arduino (через SerialTransport)
  - периодическое чтение телеметрии и публикацию /encoder_report
  - подписку на /wheel_velocity_command и отправку команд на Arduino
  - корректную остановку моторов при завершении ноды

Все параметры берутся из config/arduino_bridge.yaml.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
try:
    from rtk2026_interfaces.msg import EncoderReport, MotorCommand, WheelVelocityCommand
    HAS_WHEEL_VELOCITY_MSG = True
except ImportError:
    from rtk2026_interfaces.msg import EncoderReport, MotorCommand
    WheelVelocityCommand = None
    HAS_WHEEL_VELOCITY_MSG = False

from rtk2026_driver.protocol import InvalidChecksumError, pack_command, parse_telemetry
from rtk2026_driver.transport import HandshakeTimeoutError, SerialTransport
from rtk2026_interfaces.msg import EncoderReport, WheelVelocityCommand


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
        self.declare_parameter("wheel_velocity_command_topic", "wheel_velocity_command")
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

        # инверсия знака через yaml — не трогаем прошивку Arduino
        self._left_enc_sign = (
            -1 if self.get_parameter("left_encoder_inverted").value else 1
        )
        self._right_enc_sign = (
            -1 if self.get_parameter("right_encoder_inverted").value else 1
        )
        self._left_mot_sign = (
            -1 if self.get_parameter("left_wheel_inverted").value else 1
        )
        self._right_mot_sign = (
            -1 if self.get_parameter("right_wheel_inverted").value else 1
        )

        # --- serial транспорт ---
        self._transport = SerialTransport(
            port=port,
            baudrate=baudrate,
            handshake_timeout_sec=hs_timeout,
            handshake_poll_interval_sec=hs_poll,
        )
        self._rx_buf = bytearray()
        self._last_motor_send_time = 0.0
        self._pending_payload = None
        self._has_pending = False
        self._telemetry_rx = bytearray()

        # --- публикатор /encoder_report ---
        enc_topic = self.get_parameter("encoder_report_topic").value
        self._encoder_pub = self.create_publisher(EncoderReport, enc_topic, 10)

        # --- подписка на /wheel_velocity_command ---
        cmd_topic = self.get_parameter("wheel_velocity_command_topic").value
        self._cmd_sub = self.create_subscription(
            WheelVelocityCommand,
            cmd_topic,
            self._on_wheel_velocity_command,
            10,
        )

        # --- таймер чтения телеметрии ---
        # вызывается часто чтобы не накапливать данные в буфере
        self._read_timer = self.create_timer(0.01, self._read_telemetry)

        # --- открываем порт и ждём handshake ---
        self._open_transport(port)

    def _open_transport(self, port: str) -> None:
        """Открыть serial порт и выполнить handshake с Arduino."""
        try:
            first_frame = self._transport.open()
            self.get_logger().info(
                f"Arduino handshake successful on {port} "
                f"(left_count={first_frame.left_count}, "
                f"right_count={first_frame.right_count})"
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
            self._open_serial()
            return

        try:
            waiting = self._ser.in_waiting
            if waiting > 0:
                self._rx_buf.extend(self._ser.read(waiting))

        # извлекаем все доступные фреймы из буфера за один вызов
        consecutive_errors = 0
        max_consecutive_errors = 10

<<<<<<< HEAD
        while True:
            try:
                frame = parse_telemetry(self._rx_buf)
                consecutive_errors = 0  # сбрасываем счётчик при успехе
            except InvalidChecksumError as e:
                self.get_logger().warn(f"Telemetry checksum error: {e}")
                consecutive_errors += 1
                if consecutive_errors >= max_consecutive_errors:
                    # слишком много битых фреймов подряд — буфер рассинхронизирован
                    self.get_logger().error(
                        f"Too many consecutive checksum errors ({consecutive_errors}), clearing buffer"
                    )
                    self._rx_buf.clear()
                    break
                continue
=======
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
>>>>>>> arduino

            if frame is None:
                # буфер не накопил полный фрейм — ждём следующего вызова
                break

            msg = EncoderReport()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.left_count = self._left_enc_sign * frame.left_count
            msg.right_count = self._right_enc_sign * frame.right_count
            self._encoder_pub.publish(msg)

    def _on_wheel_velocity_command(self, msg: WheelVelocityCommand) -> None:
        """
        Подписка: получаем целевые скорости бортов и отправляем на Arduino.

        Инверсия знака мотора применяется здесь через yaml параметры —
        прошивка Arduino знаков не меняет.
        """
        left_tps = self._left_mot_sign * int(msg.left_tps)
        right_tps = self._right_mot_sign * int(msg.right_tps)
        self._transport.write(pack_command(left_tps, right_tps))

    def _stop_motors(self) -> None:
        """Отправить нулевую команду — остановить оба мотора."""
        self._transport.write(pack_command(0, 0))

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
