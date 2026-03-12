import subprocess
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_description_string() -> str:
    pkg_share = get_package_share_directory("rtk2026_simulation")
    xacro_path = Path(pkg_share) / "urdf" / "rtk2026_diff_drive_gazebo.urdf.xacro"
    config_path = Path(pkg_share) / "config" / "diff_drive_controller.yaml"
    result = subprocess.run(
        [
            "xacro",
            str(xacro_path),
            "controller_config_file:=" + str(config_path),
        ],
        capture_output=True,
        text=True,
        timeout=10,
    )
    result.check_returncode()
    return result.stdout


def generate_launch_description() -> LaunchDescription:
    # Launch-time parameters
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    R = LaunchConfiguration("R")
    P = LaunchConfiguration("P")
    Y = LaunchConfiguration("Y")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Generate URDF with ros2_control config and write it to a temp file for spawn_entity.py
    robot_description_xml = _robot_description_string()
    robot_description = {"robot_description": robot_description_xml}

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", delete=False, prefix="rtk2026_gazebo_"
    ) as f:
        f.write(robot_description_xml)
        urdf_path = f.name

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    gazebo_launch = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        output="screen",
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "gazebo_ros",
            "spawn_entity.py",
            "-entity",
            "rtk2026",
            "-file",
            urdf_path,
            "-timeout",
            "120.0",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            R,
            "-P",
            P,
            "-Y",
            Y,
        ],
        output="screen",
    )

    load_joint_state = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    load_diff_drive = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "diff_drive_controller",
        ],
        output="screen",
    )
    activate_joint_state = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_controller_state",
            "joint_state_broadcaster",
            "start",
        ],
        output="screen",
    )
    activate_diff_drive = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_controller_state",
            "diff_drive_controller",
            "start",
        ],
        output="screen",
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
                description="Use simulation time for all nodes.",
            ),
            robot_state_publisher,
            gazebo_launch,
            # Heavy worlds (small_city) need time to start before spawn_entity
            TimerAction(
                period=20.0,
                actions=[spawn_entity],
            ),
            # Load and start ros2_control controllers after Gazebo and robot are up
            TimerAction(
                period=30.0,
                actions=[
                    load_joint_state,
                    load_diff_drive,
                    activate_joint_state,
                    activate_diff_drive,
                ],
            ),
        ]
    )

