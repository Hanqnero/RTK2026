from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arduino_cfg = PathJoinSubstitution(
        [FindPackageShare("rtk2026_driver"), "config", "arduino_bridge.yaml"]
    )
    odometry_cfg = PathJoinSubstitution(
        [FindPackageShare("rtk2026_odometry"), "config", "odometry.yaml"]
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("rtk2026_description"), "launch", "description.launch.py"]
                )
            ]
        )
    )

    arduino_bridge = Node(
        package="rtk2026_driver",
        executable="arduino_bridge",
        parameters=[arduino_cfg],
        output="screen",
    )

    wheel_odometry = Node(
        package="rtk2026_odometry",
        executable="wheel_odometry",
        parameters=[odometry_cfg],
        output="screen",
    )

    return LaunchDescription(
        [
            description_launch,
            arduino_bridge,
            wheel_odometry,
        ]
    )
