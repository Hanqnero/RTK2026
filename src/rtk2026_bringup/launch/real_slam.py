"""Составной запуск описания, датчиков, драйвера и SLAM реального робота."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Сформировать полный стек картографирования на реальном роботе."""

    use_rviz = LaunchConfiguration("use_rviz")
    use_imu = LaunchConfiguration("use_imu")
    ekf_config = LaunchConfiguration("ekf_config")

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
    # base_footprint → base_link → lidar_link → lidar_frame;
    #                              → imu_link;
    #                              → camera_link → camera_optical_frame.
    #
    # use_visual:=true, хотя самому роботу геометрия не нужна.
    #
    # Её потребитель - RViz на ноутбуке, и берёт он её из топика
    # /robot_description, а не из файла: так устроены все конфиги RViz
    # в проекте (Description Source: Topic, Transient Local). Топик
    # латчится, поэтому RViz получает модель даже подключившись позже.
    #
    # Цена мала: вся геометрия - четыре коробки и три цилиндра, внешних
    # мешей в описании нет, поэтому строка остаётся самодостаточной и
    # не тянет за собой файлы, которых на роботе нет.
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
            "use_visual": "true",
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

    # Запуск лидара. Модель берётся по умолчанию из lidar_launch.py, то
    # есть C1; запасной A1M8 включается там аргументом model:=a1.
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

    # BMI270 по I2C самой Raspberry Pi. Отключается через use_imu:=false,
    # и тогда EKF надо запускать с ekf_real_wheel_only.yaml: штатный
    # ekf_real.yaml берёт курсовую скорость именно отсюда.
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

    # Локальный EKF реального робота.
    #
    # Arduino bridge публикует сырую /wheel/odom без TF. Отдельная нода
    # Raspberry Pi должна публиковать BMI270 как /imu/data в imu_link.
    # EKF использует vx, vy=0 и gyro Z, затем публикует
    # /odometry/filtered и единственный TF odom -> base_footprint.
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

    # Запуск построения карты.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    bringup_launch_directory,
                    "slam_launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    # RViz необязателен: на Raspberry Pi стек запускается headless, а на ПК
    # его можно включить аргументом use_rviz:=true.
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
            slam_launch,
            rviz,
        ]
    )
