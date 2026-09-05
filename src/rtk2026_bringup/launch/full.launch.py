"""Bring up the complete hardware layer of the real RTK2026 robot.

This launch file deliberately stops at hardware drivers and the static robot
description.  Localization, mapping, navigation, perception, diagnostics and
visualization are runtime choices and must be started separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Start the real robot description and all on-board hardware drivers."""

    use_arduino = LaunchConfiguration("use_arduino")
    use_lidar = LaunchConfiguration("use_lidar")
    use_imu = LaunchConfiguration("use_imu")
    lidar_model = LaunchConfiguration("lidar_model")

    bringup_launch_directory = PathJoinSubstitution(
        [FindPackageShare("rtk2026_bringup"), "launch"]
    )

    # This is infrastructure rather than an algorithm: it publishes the fixed
    # base and sensor transforms, plus the latched robot description used by a
    # remote RViz instance.  The camera frame is useful even though the camera
    # driver has its own deployment lifecycle.
    description = IncludeLaunchDescription(
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

    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_launch_directory, "arduino_launch.py"]
            )
        ),
        condition=IfCondition(use_arduino),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_launch_directory, "lidar_launch.py"])
        ),
        launch_arguments={"model": lidar_model}.items(),
        condition=IfCondition(use_lidar),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_launch_directory, "imu_launch.py"])
        ),
        condition=IfCondition(use_imu),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_arduino",
                default_value="true",
                description="Start the Arduino drive and wheel odometry bridge.",
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="true",
                description="Start the physical RPLIDAR driver.",
            ),
            DeclareLaunchArgument(
                "use_imu",
                default_value="true",
                description="Start the Raspberry Pi BMI270 driver.",
            ),
            DeclareLaunchArgument(
                "lidar_model",
                default_value="c1",
                choices=["c1", "a1"],
                description="Installed RPLIDAR model; a1 selects the spare A1M8.",
            ),
            description,
            drive,
            lidar,
            imu,
        ]
    )
