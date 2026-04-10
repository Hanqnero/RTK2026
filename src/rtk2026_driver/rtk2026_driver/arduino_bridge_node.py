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
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from rtk2026_interfaces.msg import EncoderReport, WheelVelocityCommand
from rtk2026_driver.protocol import pack_command, parse_telemetry, InvalidChecksumError
from rtk2026_driver.transport import SerialTransport, HandshakeTimeoutError


class ArduinoBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__("arduino_bridge")

        # --- параметры из arduino_bridge.yaml ---
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("handshake_timeout_sec", 5.0)
        self.declare_parameter("handshake_poll_interval_sec", 0.01)
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("wheel_velocity_command_topic", "wheel_velocity_command")
        self.declare_parameter("left_encoder_inverted", False)
        self.declare_parameter("right_encoder_inverted", False)
        self.declare_parameter("left_wheel_inverted", False)
        self.declare_parameter("right_wheel_inverted", False)

        port       = self.get_parameter("serial_port").value
        baudrate   = self.get_parameter("baudrate").value
        hs_timeout = self.get_parameter("handshake_timeout_sec").value
        hs_poll    = self.get_parameter("handshake_poll_interval_sec").value

        # инверсия знака через yaml — не трогаем прошивку Arduino
        self._left_enc_sign  = -1 if self.get_parameter("left_encoder_inverted").value  else 1
        self._right_enc_sign = -1 if self.get_parameter("right_encoder_inverted").value else 1
        self._left_mot_sign  = -1 if self.get_parameter("left_wheel_inverted").value    else 1
        self._right_mot_sign = -1 if self.get_parameter("right_wheel_inverted").value   else 1

        # --- serial транспорт ---
        self._transport = SerialTransport(
            port=port,
            baudrate=baudrate,
            handshake_timeout_sec=hs_timeout,
            handshake_poll_interval_sec=hs_poll,
        )
        self._rx_buf = bytearray()

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
        except HandshakeTimeoutError as e:
            # нода не может работать без Arduino — завершаемся
            self.get_logger().fatal(f"Handshake failed: {e}")
            raise SystemExit(1)

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

        # извлекаем все доступные фреймы из буфера за один вызов
        consecutive_errors = 0
        max_consecutive_errors = 10

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

            if frame is None:
                # буфер не накопил полный фрейм — ждём следующего вызова
                break

            msg = EncoderReport()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.left_count  = self._left_enc_sign  * frame.left_count
            msg.right_count = self._right_enc_sign * frame.right_count
            self._encoder_pub.publish(msg)

    def _on_wheel_velocity_command(self, msg: WheelVelocityCommand) -> None:
        """
        Подписка: получаем целевые скорости бортов и отправляем на Arduino.

        Инверсия знака мотора применяется здесь через yaml параметры —
        прошивка Arduino знаков не меняет.
        """
        left_tps  = self._left_mot_sign  * int(msg.left_tps)
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
