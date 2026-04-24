from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_rviz = PathJoinSubstitution(
        [FindPackageShare("rtk2026_bringup"), "rviz", "rtk2026_real_robot.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config for monitoring the real RTK2026 robot from a Linux workstation.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Keep false for the real robot. Only set true for recorded/simulated playback.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rtk2026_remote_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
