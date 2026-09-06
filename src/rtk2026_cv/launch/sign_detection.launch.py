"""Камера и детектор знаков на одной машине.

Оба узла поднимаются вместе, потому что разносить их нельзя: между ними идёт
поток кадров, и в сети он стоил бы дороже всего остального обмена вместе
взятого. Наружу уходит только ``DrivingDetection`` - несколько десятков байт
на сообщение, их принимает ``city_nav`` на другой машине.

Имена узлов совпадают с ключами в файле параметров, иначе ROS не отдаст им
их секции.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("rtk2026_cv")
    params_file = LaunchConfiguration("params_file")
    model_path = LaunchConfiguration("model_path")

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[params_file],
        # Драйвер публикует image_raw в своём пространстве имён, а детектор
        # ждёт топик с префиксом камеры.
        remappings=[
            ("/image_raw", "/camera/image_raw"),
            ("/camera_info", "/camera/camera_info"),
        ],
        on_exit=EmitEvent(
            event=Shutdown(reason="camera or sign detector exited")
        ),
    )

    detector = Node(
        package="rtk2026_cv",
        executable="onnx_sign_detector",
        name="onnx_sign_detector",
        output="screen",
        parameters=[params_file, {"model_path": model_path}],
        on_exit=EmitEvent(
            event=Shutdown(reason="camera or sign detector exited")
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        share,
                        "config",
                        "sign_detection_pi4.yaml",
                    ]
                ),
                description=(
                    "Параметры камеры и детектора. Один файл на оба узла: "
                    "разрешение съёмки и разрешение входа модели связаны."
                ),
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution([share, "best.onnx"]),
                description="Installed ONNX model.",
            ),
            camera,
            detector,
        ]
    )
