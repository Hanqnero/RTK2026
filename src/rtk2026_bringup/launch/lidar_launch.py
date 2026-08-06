"""Запуск драйвера RPLIDAR C1 с параметрами реального подключения."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Подключить официальный launch ``sllidar_ros2`` для RPLIDAR C1."""

    # Путь к официальному launch-файлу RPLIDAR C1.
    lidar_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("sllidar_ros2"),
            "launch",
            "sllidar_c1_launch.py",
        ]
    )

    # Включаем launch-файл пакета sllidar_ros2,
    # но заменяем нужные параметры своими значениями.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            lidar_launch_file
        ),
        launch_arguments={
            # Используем serial-соединение.
            "channel_type": "serial",

            # Путь к лидару внутри Docker.
            "serial_port": "/dev/rplidar",

            # Скорость RPLIDAR C1.
            "serial_baudrate": "460800",

            # Должно совпадать с link в URDF.
            "frame_id": "lidar_frame",

            # Лидар установлен обычной стороной вверх.
            "inverted": "false",

            # Включаем угловую компенсацию драйвера.
            "angle_compensate": "true",

            # Стандартный режим сканирования.
            "scan_mode": "Standard",
        }.items(),
    )

    return LaunchDescription(
        [
            lidar_launch,
        ]
    )
    
