from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("rtk2026_cv"), "config", "yolo_sign_adapter.yaml"]
                ),
            ),
            Node(
                package="rtk2026_cv",
                executable="yolo_sign_adapter",
                name="yolo_sign_adapter",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
