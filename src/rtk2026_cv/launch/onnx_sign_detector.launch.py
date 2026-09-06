from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("rtk2026_cv")
    params_file = LaunchConfiguration("params_file")
    model_path = LaunchConfiguration("model_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "onnx_sign_detector.yaml"]
                ),
                description="Detector parameters.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution([share, "best.onnx"]),
                description="Installed ONNX model.",
            ),
            Node(
                package="rtk2026_cv",
                executable="onnx_sign_detector",
                name="onnx_sign_detector",
                output="screen",
                parameters=[params_file, {"model_path": model_path}],
            )
        ]
    )
