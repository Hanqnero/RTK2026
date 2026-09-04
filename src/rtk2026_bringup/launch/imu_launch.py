"""Запуск IMU bridge с конфигурацией из пакета драйвера."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Сформировать запуск единственной ноды ``imu_bridge``.

    Нода читает BMI270 по I2C самой Raspberry Pi. Робот при старте
    обязан стоять: первым делом оценивается ноль гироскопа.
    """

    # Конфигурация принадлежит драйверу и устанавливается вместе с ним.
    driver_directory = Path(get_package_share_directory("rtk2026_driver"))

    config_file = driver_directory / "config" / "imu_bridge.yaml"

    imu_bridge_node = Node(
        package="rtk2026_driver",
        executable="imu_bridge",
        # Имя ноды должно совпадать с верхним ключом YAML.
        name="imu_bridge",
        output="screen",
        parameters=[str(config_file)],
    )

    return LaunchDescription([imu_bridge_node])
