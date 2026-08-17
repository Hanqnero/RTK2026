"""Запуск интерактивной настройки perspective warp и HSV-фильтра."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Создать launch без запуска камеры или симуляции второй раз."""
    params_file = LaunchConfiguration("params_file")
    image_topic = LaunchConfiguration("image_topic")
    show_gui = LaunchConfiguration("show_gui")
    config_output_path = LaunchConfiguration("config_output_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rtk2026_cv"),
                        "config",
                        "perspective_hsv.yaml",
                    ]
                ),
                description="Начальный ROS 2 parameters YAML.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/color/image_raw",
                description="Исходный RGB-топик камеры.",
            ),
            DeclareLaunchArgument(
                "show_gui",
                default_value="true",
                description="Показывать окна OpenCV с ползунками.",
            ),
            DeclareLaunchArgument(
                "config_output_path",
                default_value=(
                    "/workspace/records/cv/perspective_hsv_tuned.yaml"
                ),
                description="Путь сохранения настройки по клавише s.",
            ),
            Node(
                package="rtk2026_cv",
                executable="perspective_hsv_tuner",
                name="perspective_hsv_tuner",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "image_topic": image_topic,
                        "show_gui": ParameterValue(
                            show_gui,
                            value_type=bool,
                        ),
                        "config_output_path": config_output_path,
                    },
                ],
            ),
        ]
    )
