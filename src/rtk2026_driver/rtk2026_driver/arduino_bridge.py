"""ROS 2-нода, связывающая ``/cmd_vel`` и Arduino.

Нода выполняет следующие задачи:

#. Получает :class:`geometry_msgs.msg.TwistStamped` из ``/cmd_vel``.
#. Периодически передаёт последнюю команду Arduino.
#. Читает :class:`~rtk2026_driver.protocol.TelemetryPacket` из USB Serial.
#. Публикует сырую колёсную одометрию ``/wheel/odom``.

Динамическую трансформацию ``odom -> base_footprint`` по умолчанию публикует
EKF. Bridge может публиковать её сам только в аварийном режиме без фильтра.
"""

import math
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from rtk2026_driver.protocol import (
    TelemetryPacket,
    pack_command,
    pop_telemetry_packet,
)
from rtk2026_driver.transport import SerialTransport


class ArduinoBridgeNode(Node):
    """Мост между ROS2-топиками и USB Serial Arduino."""

    def __init__(self) -> None:
        """Создать ROS2-ноду, открыть порт и запустить таймеры."""

        super().__init__("arduino_bridge")

        self.declare_parameter("serial_port", "/dev/arduino") # задание порта
        self.declare_parameter("baudrate", 115200) # скорость serial-порта (в боудах)
        self.declare_parameter("arduino_reset_wait_sec", 1.0) # время ожидания после открытия порта

        self.declare_parameter("cmd_vel_topic", "/cmd_vel") # входной топик данной ноды (по умолч "/cmd_vel")
        self.declare_parameter("odom_topic", "/wheel/odom") # выходной топик данной ноды

        self.declare_parameter("odom_frame", "odom") # создаем неподвижную СК одометрии
        self.declare_parameter("base_frame", "base_footprint") # создаем СК робота
        self.declare_parameter("publish_odom_tf", False) # TF обычно принадлежит EKF

        # Диагонали ковариаций задаются отдельно от прошивки: это оценка
        # неопределённости энкодерной одометрии, а не геометрия робота.
        self.declare_parameter(
            "pose_covariance_diagonal",
            [0.02, 0.02, 1.0e6, 1.0e6, 1.0e6, 0.05],
        )
        self.declare_parameter(
            "twist_covariance_diagonal",
            [0.01, 0.01, 1.0e6, 1.0e6, 1.0e6, 0.03],
        )

        self.declare_parameter("command_send_interval_sec", 0.02) # интервал отправки команд на ардуино (в сек)
        self.declare_parameter("telemetry_poll_period_sec", 0.01) # интервал проверки буфера (в сек)
        self.declare_parameter("drop_stale_cmd_after_sec", 0.30) # макс время последней команды (иначе отправляем нулевые скорости)

        self.declare_parameter("max_linear_mps", 1.5)
        self.declare_parameter("max_angular_rps", math.pi / 2.0)
        self.declare_parameter("debug_raw_encoder", False)

        # тут мы просто приводим нодовские параметры в стр питона
        serial_port = str(
            self.get_parameter("serial_port").value
        )
        baudrate = int(
            self.get_parameter("baudrate").value
        )
        reset_wait_sec = float(
            self.get_parameter("arduino_reset_wait_sec").value
        )

        cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )
        odom_topic = str(
            self.get_parameter("odom_topic").value
        )

        self._odom_frame = str(
            self.get_parameter("odom_frame").value
        )
        self._base_frame = str(
            self.get_parameter("base_frame").value
        )
        self._publish_odom_tf = bool(
            self.get_parameter("publish_odom_tf").value
        )
        self._pose_covariance_diagonal = self._read_covariance_diagonal(
            "pose_covariance_diagonal"
        )
        self._twist_covariance_diagonal = self._read_covariance_diagonal(
            "twist_covariance_diagonal"
        )

        command_send_interval_sec = float(
            self.get_parameter("command_send_interval_sec").value
        )
        telemetry_poll_period_sec = float(
            self.get_parameter("telemetry_poll_period_sec").value
        )

        self._drop_stale_cmd_after_sec = float(
            self.get_parameter("drop_stale_cmd_after_sec").value
        )
        self._max_linear_mps = float(
            self.get_parameter("max_linear_mps").value
        )
        self._max_angular_rps = float(
            self.get_parameter("max_angular_rps").value
        )
        self._debug_raw_encoder = bool(
            self.get_parameter("debug_raw_encoder").value
        )

        self._target_linear_mps = 0.0
        self._target_angular_rps = 0.0
        self._last_cmd_time: float | None = None

        self._receive_buffer = bytearray()
        # создаем сериал объект
        self._transport = SerialTransport(
            port=serial_port,
            baudrate=baudrate,
            reset_wait_sec=reset_wait_sec,
        )
        self._transport.open()

        self._odom_publisher = self.create_publisher(
            Odometry,
            odom_topic,
            10, #глубина сообщений
        )

        self._tf_broadcaster = TransformBroadcaster(self) # Создаётся broadcaster динамических TF (вычисляет дельты между СК)

        self._cmd_vel_subscription = self.create_subscription(
            TwistStamped, # единый stamped-интерфейс реального робота и симуляции
            cmd_vel_topic,
            self._on_cmd_vel, # callback
            1,
        )
        # создаем росовский таймер отправки команд
        self._command_timer = self.create_timer(
            command_send_interval_sec,
            self._send_command,
        )
        # создаем росовский таймер отправки чтения буфера
        self._telemetry_timer = self.create_timer(
            telemetry_poll_period_sec,
            self._read_telemetry,
        )

        self.get_logger().info(
            f"Arduino bridge started on {serial_port}"
        )

    def _read_covariance_diagonal(self, parameter_name: str) -> tuple[float, ...]:
        """Прочитать и проверить шесть диагональных элементов ковариации.

        Порядок значений: ``x, y, z, roll, pitch, yaw`` для pose и
        ``vx, vy, vz, vroll, vpitch, vyaw`` для twist.

        :param parameter_name: имя ROS-параметра с массивом из шести чисел.
        :raises ValueError: если размер, знак или конечность значений неверны.
        """

        diagonal = tuple(
            float(value)
            for value in self.get_parameter(parameter_name).value
        )

        if len(diagonal) != 6:
            raise ValueError(
                f"{parameter_name} must contain exactly 6 values"
            )

        if any(
            value < 0.0 or not math.isfinite(value)
            for value in diagonal
        ):
            raise ValueError(
                f"{parameter_name} must contain finite non-negative values"
            )

        return diagonal
    # тот самый колбек подписки
    def _on_cmd_vel(self, message: TwistStamped) -> None:
        """Проверить и сохранить новую команду скорости.

        Используются только ``twist.linear.x`` и ``twist.angular.z``.
        Значения NaN и infinity отклоняются, остальные ограничиваются
        параметрами ``max_linear_mps`` и ``max_angular_rps``. Время получения
        фиксируется монотонными часами для защиты dead-man.

        :param message: команда скорости в системе координат основания.
        """

        # Время в header задаёт источник команды. Для dead-man ниже намеренно
        # используем локальное monotonic-время получения: старое или неверное
        # ROS-время не должно отключать защитную остановку.
        linear_mps = float(message.twist.linear.x)
        angular_rps = float(message.twist.angular.z)

        if not math.isfinite(linear_mps):
            self.get_logger().error(
                "Rejected non-finite linear.x"
            )
            return

        if not math.isfinite(angular_rps):
            self.get_logger().error(
                "Rejected non-finite angular.z"
            )
            return

        self._target_linear_mps = max(
            -self._max_linear_mps,
            min(linear_mps, self._max_linear_mps),
        )

        self._target_angular_rps = max(
            -self._max_angular_rps,
            min(angular_rps, self._max_angular_rps),
        )

        self._last_cmd_time = time.monotonic()
    # калбек отправки команд
    def _send_command(self) -> None:
        """Передать Arduino последнюю допустимую команду скорости.

        Если команда не поступала дольше ``drop_stale_cmd_after_sec``, вместо
        неё передаётся безопасная команда остановки.
        """

        now = time.monotonic()

        command_is_stale = (
            self._last_cmd_time is None
            or now - self._last_cmd_time
            > self._drop_stale_cmd_after_sec
        )

        if command_is_stale:
            linear_mps = 0.0
            angular_rps = 0.0
        else:
            linear_mps = self._target_linear_mps
            angular_rps = self._target_angular_rps

        command_packet = pack_command(
            linear_mps=linear_mps,
            angular_rps=angular_rps,
            debug_raw_encoder=self._debug_raw_encoder,
        )

        try:
            self._transport.write(command_packet)
        except serial.SerialException as exception:
            self._handle_serial_error(
                "write",
                exception,
            )
    # калбек чтения буфера
    def _read_telemetry(self) -> None:
        """Прочитать доступные байты и обработать все полные телепакеты.

        Неполный хвост остаётся в ``_receive_buffer`` до следующего вызова
        таймера. Ошибка serial считается фатальной для ноды.
        """

        try:
            received_bytes = self._transport.read_available()
        except serial.SerialException as exception:
            self._handle_serial_error(
                "read",
                exception,
            )
            return

        if received_bytes:
            self._receive_buffer.extend(received_bytes)

        while True:
            telemetry = pop_telemetry_packet(
                self._receive_buffer
            )

            if telemetry is None:
                break

            self._publish_odometry(telemetry)

    # ~ распаковка одометрии с ардуинки и ее публкикация
    def _publish_odometry(
        self,
        telemetry: TelemetryPacket,
    ) -> None:
        """Опубликовать сырую одометрию и, при необходимости, динамическую TF.

        Плоский угол курса из телеметрии переводится в quaternion вращения
        вокруг оси Z. При ``publish_odom_tf=false`` TF не отправляется:
        единственным владельцем ``odom -> base_footprint`` остаётся EKF.

        :param telemetry: разобранный пакет состояния Arduino.
        """

        stamp = self.get_clock().now().to_msg() # получаем текущее время

        # вычисление поворота
        half_yaw = telemetry.odom_heading_rad * 0.5
        quaternion_z = math.sin(half_yaw)
        quaternion_w = math.cos(half_yaw)

        # создание сообщения одометри
        odom_message = Odometry()

        odom_message.header.stamp = stamp # заполняем время
        odom_message.header.frame_id = self._odom_frame # заполняем СК одометрии
        odom_message.child_frame_id = self._base_frame # заполняем СК робота

        odom_message.pose.pose.position.x = telemetry.odom_x_m
        odom_message.pose.pose.position.y = telemetry.odom_y_m
        odom_message.pose.pose.position.z = 0.0

        odom_message.pose.pose.orientation.x = 0.0
        odom_message.pose.pose.orientation.y = 0.0
        odom_message.pose.pose.orientation.z = quaternion_z
        odom_message.pose.pose.orientation.w = quaternion_w

        odom_message.twist.twist.linear.x = (
            telemetry.current_linear_mps
        )
        odom_message.twist.twist.linear.y = 0.0
        odom_message.twist.twist.linear.z = 0.0

        odom_message.twist.twist.angular.x = 0.0
        odom_message.twist.twist.angular.y = 0.0
        odom_message.twist.twist.angular.z = (
            telemetry.current_angular_rps
        )

        for index, value in enumerate(self._pose_covariance_diagonal):
            odom_message.pose.covariance[index * 7] = value

        for index, value in enumerate(self._twist_covariance_diagonal):
            odom_message.twist.covariance[index * 7] = value

        self._odom_publisher.publish(odom_message) # публикация сообщения

        if not self._publish_odom_tf:
            return

        transform = TransformStamped() # создаем дерево трансформаций (чтобы в рвиз визуализировать собственно локализацию робота)

        transform.header.stamp = stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame

        transform.transform.translation.x = telemetry.odom_x_m
        transform.transform.translation.y = telemetry.odom_y_m
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w

        self._tf_broadcaster.sendTransform(transform) # публикации трансформации

    # Обработка ошибки serial
    def _handle_serial_error(
        self,
        operation: str,
        exception: Exception,
    ) -> None:
        """Закрыть транспорт и завершить ROS-контекст после ошибки serial.

        :param operation: операция, на которой произошла ошибка — чтение или
            запись.
        :param exception: исходное исключение транспорта.
        """

        self.get_logger().fatal(
            f"Serial {operation} failed: {exception}"
        )

        self._transport.close()

        if rclpy.ok():
            rclpy.shutdown()

    def destroy_node(self) -> None:
        """Остановить приводы, закрыть serial-порт и уничтожить ноду."""

        stop_packet = pack_command(
            linear_mps=0.0,
            angular_rps=0.0,
            debug_raw_encoder=False,
        )

        try:
            self._transport.write(stop_packet)
        except serial.SerialException:
            pass

        self._transport.close()

        super().destroy_node()


def main(args=None) -> None:
    """Запустить Arduino bridge и корректно освободить ресурсы при остановке."""

    rclpy.init(args=args)

    node = ArduinoBridgeNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
