"""Камера и детектор знаков на одной машине.

Оба узла поднимаются вместе, потому что разносить их нельзя: между ними идёт
поток кадров, и в сети он стоил бы дороже всего остального обмена вместе
взятого. Наружу уходит только ``DrivingDetection`` - несколько десятков байт
на сообщение, их принимает ``city_nav`` на другой машине.

Имена узлов совпадают с ключами в файле параметров, иначе ROS не отдаст им
их секции.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")

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
    )

    detector = Node(
        package="rtk2026_cv",
        executable="onnx_sign_detector",
        name="onnx_sign_detector",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("rtk2026_cv"),
                        "config",
                        "sign_detection_pi4.yaml",
                    ]
                ),
                description=(
                    "Параметры камеры и детектора. Один файл на оба узла: "
                    "разрешение съёмки и разрешение входа модели связаны."
                ),
            ),
            camera,
            detector,
        ]
    )
