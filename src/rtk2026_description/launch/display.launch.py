"""Публикация статического TF-дерева реального робота."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Развернуть Xacro и запустить ``robot_state_publisher``.

    Запускается только на роботе. На ноутбуке второй такой источник
    начнёт публиковать те же статические TF, и TF2 будет дёргаться между
    двумя одинаковыми деревьями. Модель для RViz подключается там
    напрямую файлом, см. use_visual в rtk2026_real.urdf.xacro.
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_camera = LaunchConfiguration("use_camera")
    use_visual = LaunchConfiguration("use_visual")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rtk2026_description"), "urdf", "rtk2026_real.urdf.xacro"]
    )

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_camera:=",
                use_camera,
                " use_visual:=",
                use_visual,
            ]
        ),
        value_type=str,
    )

    # Все joints фиксированные, joint_state_publisher не нужен.
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "use_camera",
                default_value="true",
                description="Добавить camera_link и camera_optical_frame.",
            ),
            DeclareLaunchArgument(
                "use_visual",
                default_value="false",
                description=(
                    "Вложить геометрию в /robot_description. На роботе не "
                    "нужна: рисует RViz на ноутбуке."
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": use_sim_time}
                ],
            ),
        ]
    )
