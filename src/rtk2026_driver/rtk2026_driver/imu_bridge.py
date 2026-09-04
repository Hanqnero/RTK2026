"""ROS 2-нода, публикующая BMI270 как ``sensor_msgs/msg/Imu``.

Нода делает ровно три вещи:

#. Поднимает BMI270 на шине I2C Raspberry Pi.
#. Оценивает смещение нуля гироскопа на неподвижном роботе.
#. Публикует ``/imu/data`` с ``header.frame_id=imu_link``.

Оси публикуются СЫРЫМИ, как их отдаёт датчик. Разворот в систему
координат робота выполняет ``robot_localization`` по TF из ``imu_rpy``
в URDF. Поправлять знаки здесь нельзя: получится двойной разворот, и
ошибка проявится только на поворотах, когда карта уже поедет.

Ориентация не публикуется. У BMI270 нет ни магнитометра, ни встроенного
слияния, поэтому абсолютный курс взять неоткуда, и
``orientation_covariance[0]`` выставлен в -1: по соглашению
``sensor_msgs/msg/Imu`` это означает "ориентации нет". EKF реального
робота её и не спрашивает, в ``ekf_real.yaml`` из всей IMU включён
единственный элемент - ``angular_velocity.z``.
"""

import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from rtk2026_driver.bmi270 import Bmi270, Bmi270Error


class ImuBridgeNode(Node):
    """Мост между BMI270 на I2C и топиком ``/imu/data``."""

    def __init__(self) -> None:
        """Создать ноду, поднять чип и запустить таймер публикации."""

        super().__init__("imu_bridge")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x68)

        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("frame_id", "imu_link")

        # Частота публикации и частота выдачи данных самим чипом заданы
        # раздельно намеренно. Публиковать чаще, чем меряет датчик,
        # бессмысленно: EKF получал бы повторы одного отсчёта и считал
        # бы их независимыми измерениями.
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("sensor_odr_hz", 100)

        self.declare_parameter("accel_range_g", 4)
        self.declare_parameter("gyro_range_dps", 1000)

        # Калибровка нуля гироскопа при старте. Смещение BMI270
        # составляет единицы градусов в секунду, а EKF его интегрирует:
        # некомпенсированный ноль превращается в равномерный уход курса
        # даже у стоящего робота.
        self.declare_parameter("calibrate_gyro_bias", True)
        self.declare_parameter("gyro_bias_samples", 200)

        # Порог отбраковки калибровки. Если робот при старте двигали,
        # усреднение даст не ноль, а скорость движения.
        self.declare_parameter("gyro_bias_max_rps", 0.20)

        self.declare_parameter(
            "angular_velocity_covariance_diagonal", [0.0004, 0.0004, 0.0004]
        )
        self.declare_parameter(
            "linear_acceleration_covariance_diagonal", [0.04, 0.04, 0.04]
        )

        # Пустая строка означает штатный файл из share пакета.
        self.declare_parameter("config_file", "")

        self.declare_parameter("link_report_period_sec", 10.0)

        i2c_bus = int(self.get_parameter("i2c_bus").value)
        i2c_address = int(self.get_parameter("i2c_address").value)

        imu_topic = str(self.get_parameter("imu_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        sensor_odr_hz = int(self.get_parameter("sensor_odr_hz").value)

        accel_range_g = int(self.get_parameter("accel_range_g").value)
        gyro_range_dps = int(self.get_parameter("gyro_range_dps").value)

        self._angular_velocity_covariance = self._read_covariance_diagonal(
            "angular_velocity_covariance_diagonal"
        )
        self._linear_acceleration_covariance = self._read_covariance_diagonal(
            "linear_acceleration_covariance_diagonal"
        )

        link_report_period_sec = float(
            self.get_parameter("link_report_period_sec").value
        )

        if publish_rate_hz > sensor_odr_hz:
            self.get_logger().warning(
                f"publish_rate_hz={publish_rate_hz} выше частоты датчика "
                f"{sensor_odr_hz} Гц: часть сообщений повторит один отсчёт"
            )

        config_blob = self._load_configuration()

        self._sensor = Bmi270(
            bus=i2c_bus,
            address=i2c_address,
            accel_range_g=accel_range_g,
            gyro_range_dps=gyro_range_dps,
            output_data_rate_hz=sensor_odr_hz,
        )
        self._sensor.open(config_blob)

        self.get_logger().info(
            f"BMI270 поднят на /dev/i2c-{i2c_bus} по адресу 0x{i2c_address:02X}, "
            f"±{accel_range_g}g / ±{gyro_range_dps}°/с, {sensor_odr_hz} Гц"
        )

        self._gyro_bias = self._estimate_gyro_bias()

        self._imu_publisher = self.create_publisher(Imu, imu_topic, 10)

        self._published = 0
        self._read_errors = 0

        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish_sample)
        self._report_timer = self.create_timer(
            link_report_period_sec, self._report_health
        )

    def _read_covariance_diagonal(self, parameter_name: str) -> tuple[float, ...]:
        """Прочитать и проверить три диагональных элемента ковариации.

        :param parameter_name: имя параметра ноды.
        :raises ValueError: если элементов не три или они отрицательные.
        """

        values = tuple(float(value) for value in self.get_parameter(parameter_name).value)

        if len(values) != 3:
            raise ValueError(
                f"{parameter_name}: ожидалось 3 значения, получено {len(values)}"
            )

        if any(value < 0.0 for value in values):
            raise ValueError(f"{parameter_name}: дисперсия не может быть отрицательной")

        return values

    def _load_configuration(self) -> bytes:
        """Прочитать образ конфигурации BMI270.

        :raises FileNotFoundError: файл образа отсутствует.
        """

        configured_path = str(self.get_parameter("config_file").value)

        if configured_path:
            config_path = Path(configured_path)
        else:
            config_path = (
                Path(get_package_share_directory("rtk2026_driver"))
                / "config"
                / "bmi270_config.bin"
            )

        if not config_path.is_file():
            raise FileNotFoundError(
                f"образ конфигурации BMI270 не найден: {config_path}"
            )

        return config_path.read_bytes()

    def _estimate_gyro_bias(self) -> tuple[float, float, float]:
        """Усреднить показания неподвижного гироскопа.

        Робот при старте ноды обязан стоять. Если среднее оказалось
        больше ``gyro_bias_max_rps``, калибровка отбрасывается: такое
        значение означает, что робот двигали, и вычитать его нельзя.
        """

        if not bool(self.get_parameter("calibrate_gyro_bias").value):
            self.get_logger().info("Калибровка нуля гироскопа отключена параметром")
            return (0.0, 0.0, 0.0)

        sample_count = int(self.get_parameter("gyro_bias_samples").value)
        bias_limit_rps = float(self.get_parameter("gyro_bias_max_rps").value)

        if sample_count <= 0:
            return (0.0, 0.0, 0.0)

        self.get_logger().info(
            f"Калибровка нуля гироскопа: {sample_count} отсчётов, робот должен стоять"
        )

        sums = [0.0, 0.0, 0.0]

        for _ in range(sample_count):
            _, gyro = self._sensor.read()

            for axis in range(3):
                sums[axis] += gyro[axis]

            time.sleep(0.005)

        bias = tuple(value / sample_count for value in sums)

        if max(abs(value) for value in bias) > bias_limit_rps:
            self.get_logger().error(
                f"Смещение нуля {bias} превышает предел {bias_limit_rps} рад/с. "
                "Робот при калибровке двигался, поправка не применяется"
            )
            return (0.0, 0.0, 0.0)

        degrees = tuple(math.degrees(value) for value in bias)

        self.get_logger().info(
            f"Ноль гироскопа: "
            f"x={degrees[0]:+.3f} y={degrees[1]:+.3f} z={degrees[2]:+.3f} °/с"
        )

        return bias

    def _publish_sample(self) -> None:
        """Прочитать датчик и опубликовать сообщение."""

        try:
            accel, gyro = self._sensor.read()
        except OSError as exception:
            self._read_errors += 1

            # Ошибка шины не повод убивать ноду: I2C переживает
            # одиночные сбои, а перезапуск стека стоит дороже.
            self.get_logger().warning(f"Чтение BMI270 не удалось: {exception}")
            return

        message = Imu()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id

        # -1 в первом элементе - принятое в sensor_msgs обозначение
        # отсутствующей ориентации. Потребитель обязан её игнорировать.
        message.orientation_covariance[0] = -1.0

        message.angular_velocity.x = gyro[0] - self._gyro_bias[0]
        message.angular_velocity.y = gyro[1] - self._gyro_bias[1]
        message.angular_velocity.z = gyro[2] - self._gyro_bias[2]

        message.linear_acceleration.x = accel[0]
        message.linear_acceleration.y = accel[1]
        message.linear_acceleration.z = accel[2]

        for index, value in enumerate(self._angular_velocity_covariance):
            message.angular_velocity_covariance[index * 4] = value

        for index, value in enumerate(self._linear_acceleration_covariance):
            message.linear_acceleration_covariance[index * 4] = value

        self._imu_publisher.publish(message)
        self._published += 1

    def _report_health(self) -> None:
        """Записать в лог состояние обмена с датчиком."""

        message = f"imu: опубликовано={self._published} ошибок={self._read_errors}"

        if self._read_errors:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def destroy_node(self) -> None:
        """Закрыть шину перед уничтожением ноды."""

        try:
            self._sensor.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    """Точка входа исполняемого файла ``imu_bridge``."""

    rclpy.init(args=args)

    try:
        node = ImuBridgeNode()
    except (Bmi270Error, OSError, FileNotFoundError) as exception:
        # Без датчика нода бесполезна: EKF ждёт /imu/data и с молчащим
        # источником будет тихо деградировать вместо явной ошибки.
        rclpy.logging.get_logger("imu_bridge").fatal(f"Запуск не удался: {exception}")
        rclpy.try_shutdown()
        return

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
