"""Составной запуск описания, датчиков, драйвера и SLAM реального робота."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Сформировать полный стек картографирования на реальном роботе."""

    # Каталог launch-файлов пакета rtk2026_bringup.
    bringup_launch_directory = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_bringup"),
            "launch",
        ]
    )

    # Запуск описания реального робота.
    #
    # robot_state_publisher публикует фиксированные TF:
    # base_footprint → base_link → lidar_link → lidar_frame.
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rtk2026_description"),
                    "launch",
                    "display.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "use_meshes": "false",
            "use_webcam": "true",
        }.items(),
    )

    # Запуск связи с Arduino.
    arduino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    bringup_launch_directory,
                    "arduino_launch.py",
                ]
            )
        )
    )

    # Запуск RPLIDAR C1.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    bringup_launch_directory,
                    "lidar_launch.py",
                ]
            )
        )
    )

    # Запуск построения карты.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    bringup_launch_directory,
                    "slam_launch.py",
                ]
            )
        )
    )

    return LaunchDescription(
        [
            description_launch,
            arduino_launch,
            lidar_launch,
            slam_launch,
        ]
    )
