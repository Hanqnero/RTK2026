"""Запуск Arduino bridge с конфигурацией из пакета драйвера."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Сформировать запуск единственной ноды ``arduino_bridge``."""

    # Конфигурация принадлежит драйверу и устанавливается вместе с ним.
    # Bringup только находит её в ament index и передаёт готовой ноде.
    driver_directory = Path(
        get_package_share_directory("rtk2026_driver")
    )

    # Формируем путь к YAML-конфигурации Arduino bridge.
    config_file = (
        driver_directory
        / "config"
        / "arduino_bridge.yaml"
    )

    # Создаём описание процесса Arduino bridge.
    arduino_bridge_node = Node(
        # ROS2-пакет, в котором зарегистрирован executable.
        package="rtk2026_driver",

        # Имя executable из setup.py или CMakeLists.txt пакета driver.
        executable="arduino_bridge",

        # Имя ноды должно совпадать с верхним ключом YAML.
        name="arduino_bridge",

        # Передавать stdout и ROS-логи в консоль.
        output="screen",

        # Передаём параметры из YAML-файла.
        parameters=[
            str(config_file),
        ],
    )

    return LaunchDescription(
        [
            arduino_bridge_node,
        ]
    )
