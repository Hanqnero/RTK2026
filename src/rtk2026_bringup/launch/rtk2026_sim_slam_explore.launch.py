from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    R = LaunchConfiguration("R")
    P = LaunchConfiguration("P")
    Y = LaunchConfiguration("Y")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_launch = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_simulation"),
            "launch",
            "rtk2026_gazebo.launch.py",
        ]
    )
    nav2_slam_launch = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_nav2_explorer"),
            "launch",
            "rtk2026_nav2_slam.launch.py",
        ]
    )
    explorer_launch = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_nav2_explorer"),
            "launch",
            "rtk2026_explorer.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="empty.world",
                description="Absolute path to Gazebo world file inside container.",
            ),
            DeclareLaunchArgument(
                "x",
                default_value="0.0",
                description="Initial x position of the robot",
            ),
            DeclareLaunchArgument(
                "y",
                default_value="0.0",
                description="Initial y position of the robot",
            ),
            DeclareLaunchArgument(
                "z",
                default_value="0.15",
                description="Initial z position of the robot",
            ),
            DeclareLaunchArgument(
                "R",
                default_value="0.0",
                description="Initial roll orientation of the robot",
            ),
            DeclareLaunchArgument(
                "P",
                default_value="0.0",
                description="Initial pitch orientation of the robot",
            ),
            DeclareLaunchArgument(
                "Y",
                default_value="0.0",
                description="Initial yaw orientation of the robot",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time in Gazebo + SLAM + Nav2 + explorer.",
            ),
            # 1) Gazebo + robot + ros2_control
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "world": world,
                    "x": x,
                    "y": y,
                    "z": z,
                    "R": R,
                    "P": P,
                    "Y": Y,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            # 2) Nav2 + SLAM (after Gazebo is reasonably up)
            GroupAction(
                [
                    TimerAction(
                        period=25.0,
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(nav2_slam_launch),
                                launch_arguments={
                                    "use_sim_time": use_sim_time,
                                }.items(),
                            ),
                        ],
                    ),
                ]
            ),
            # 3) Frontier-based explorer (after Nav2/Slam are running)
            GroupAction(
                [
                    TimerAction(
                        period=40.0,
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(explorer_launch),
                                launch_arguments={
                                    "use_sim_time": use_sim_time,
                                }.items(),
                            ),
                        ],
                    ),
                ]
            ),
        ]
    )

