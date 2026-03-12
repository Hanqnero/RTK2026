# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# Gazebo + diff_drive by ros2_control (ros2_diff_drive_robot style).
# Requires: ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
#           ros-humble-ros2-control ros-humble-ros2-controllers
#           ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster

import subprocess
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def _robot_description_string():
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


def generate_launch_description():
    robot_description = _robot_description_string()

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", delete=False, prefix="rtk2026_gazebo_"
    ) as f:
        f.write(robot_description)
        urdf_path = f.name

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # Start Gazebo server with GazeboRosFactory so spawn_entity can insert the robot.
    gazebo_launch = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    spawn_entity = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "gazebo_ros",
            "spawn_entity.py",
            "-entity", "rtk2026",
            "-file", urdf_path,
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

    return LaunchDescription([
        robot_state_publisher,
        gazebo_launch,
        TimerAction(
            period=5.0,
            actions=[spawn_entity],
        ),
        TimerAction(
            period=10.0,
            actions=[
                load_joint_state,
                load_diff_drive,
                activate_joint_state,
                activate_diff_drive,
            ],
        ),
    ])
