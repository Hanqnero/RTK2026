"""Публикация URDF и статического TF-дерева реального робота."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Развернуть Xacro и запустить ``robot_state_publisher``."""

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_meshes = LaunchConfiguration("use_meshes")
    use_webcam = LaunchConfiguration("use_webcam")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("rtk2026_description"), "urdf", "rtk2026_real.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " use_meshes:=",
                use_meshes,
                " use_webcam:=",
                use_webcam,
            ]
        ),
        value_type=str,
    )

    # The real/RViz model contains only fixed joints, so joint_state_publisher is unnecessary.
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_meshes", default_value="true"),
            DeclareLaunchArgument("use_webcam", default_value="true"),
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
