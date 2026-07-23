"""Запуск локального EKF для непрерывной одометрии RTK2026.

Нода читает ``/wheel/odom``, публикует ``/odometry/filtered`` и владеет
динамической трансформацией ``odom -> base_footprint``. Глобальная
трансформация ``map -> odom`` в этот launch не входит.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Сформировать запуск ``robot_localization/ekf_node``.

    Launch arguments позволяют выбрать источник времени и отдельную
    конфигурацию измерений, не дублируя запуск ноды.
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")

    # ~ Конфигурация принадлежит пакету локализации, а не bringup.
    default_ekf_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_localization"),
            "config",
            "ekf.yaml",
        ]
    )

    # ~ Непрерывная локальная оценка движения и odom TF.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        # Явный абсолютный output сохраняет единый API без зависимости от
        # namespace, с которым позже может запускаться пакет.
        remappings=[
            (
                "odometry/filtered",
                "/odometry/filtered",
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Использовать часы из /clock.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_ekf_config,
                description="Абсолютный путь к YAML-конфигурации EKF.",
            ),
            ekf_node,
        ]
    )
