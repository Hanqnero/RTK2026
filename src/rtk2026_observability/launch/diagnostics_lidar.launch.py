"""Диагностика tracked-симуляции со SLAM Toolbox."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Подключить общую диагностику с lidar-профилем."""

    use_gui = LaunchConfiguration("use_gui")
    diagnostics_display = LaunchConfiguration("diagnostics_display")
    records_path = LaunchConfiguration("records_path")
    common_launch = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "launch",
            "diagnostics.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument(
                "diagnostics_display",
                default_value=EnvironmentVariable(
                    "DIAGNOSTICS_DISPLAY",
                    default_value=":2",
                ),
            ),
            DeclareLaunchArgument(
                "records_path",
                default_value="/workspace/records",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(common_launch),
                launch_arguments={
                    "use_gui": use_gui,
                    "diagnostics_display": diagnostics_display,
                    "localization_mode": "mapping",
                    "slam_backend": "lidar",
                    "records_path": records_path,
                }.items(),
            ),
        ]
    )
