from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rtk2026_cv",
                executable="onnx_sign_detector",
                name="onnx_sign_detector",
                output="screen",
                parameters=["/workspace/install/rtk2026_cv/share/rtk2026_cv/config/onnx_sign_detector.yaml"],
            )
        ]
    )
