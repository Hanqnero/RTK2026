"""Общий запуск SLAM Toolbox для симуляции и реального робота."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Подключить ``online_async_launch.py`` с конфигурацией проекта."""

    # Один и тот же launch используется реальным роботом и симуляцией.
    # Значение false подходит реальному роботу, а sim_slam_launch.py передаёт true.
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Параметры SLAM принадлежат пакету rtk2026_slam.
    # Bringup не хранит свою копию YAML, чтобы настройки не расходились.
    slam_directory = Path(
        get_package_share_directory("rtk2026_slam")
    )

    # Путь к нашей конфигурации slam_toolbox.
    slam_config_file = (
        slam_directory
        / "config"
        / "slam_toolbox_params.yaml"
    )

    # Путь к официальному online async launch-файлу.
    slam_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("slam_toolbox"),
            "launch",
            "online_async_launch.py",
        ]
    )

    # Включаем официальный запуск slam_toolbox.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            slam_launch_file
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,

            # Запустить, сконфигурировать и активировать ноду автоматически.
            "autostart": "true",

            # Передать нашу YAML-конфигурацию.
            "slam_params_file": str(slam_config_file),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Использовать время из Gazebo-топика /clock.",
            ),
            slam_launch,
        ]
    )
