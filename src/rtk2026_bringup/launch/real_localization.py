"""Локализация реального RTK2026 по заранее сохранённой карте."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Запустить датчики, EKF с внешней Pi IMU, Map Server и AMCL."""

    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    use_imu = LaunchConfiguration("use_imu")
    ekf_config = LaunchConfiguration("ekf_config")

    bringup_launch_directory = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_bringup"),
            "launch",
        ]
    )

    # ~ Статическая часть TF-дерева реального робота.
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
            "use_camera": "true",
        }.items(),
    )

    # ~ Команды движения и сырая энкодерная /wheel/odom.
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

    # ~ Физический RPLIDAR A1M8 публикует /scan в lidar_frame.
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

    # ~ BMI270 по I2C самой Raspberry Pi. Отключается через use_imu:=false,
    # и тогда EKF надо запускать с ekf_real_wheel_only.yaml.
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    bringup_launch_directory,
                    "imu_launch.py",
                ]
            )
        ),
        condition=IfCondition(use_imu),
    )

    # ~ EKF владеет odom -> base_footprint.
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rtk2026_localization"),
                    "launch",
                    "ekf.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "config_file": ekf_config,
        }.items(),
    )

    # ~ Map Server публикует карту, AMCL — единственный TF map -> odom.
    particle_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rtk2026_localization"),
                    "launch",
                    "particle_localization.launch.py",
                ]
            )
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("rtk2026_bringup"),
                    "rviz",
                    "rtk2026_real_robot.rviz",
                ]
            ),
        ],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Абсолютный путь к YAML сохранённой карты.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Запустить RViz на текущем X11-дисплее.",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="true",
                description=(
                    "Запустить BMI270. При false подставьте "
                    "ekf_config:=.../ekf_real_wheel_only.yaml."
                ),
            ),
            DeclareLaunchArgument(
                "ekf_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rtk2026_localization"),
                        "config",
                        "ekf_real.yaml",
                    ]
                ),
                description="Конфигурация EKF реального робота.",
            ),
            description_launch,
            arduino_launch,
            lidar_launch,
            imu_launch,
            ekf_launch,
            particle_localization,
            rviz,
        ]
    )
