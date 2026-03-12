# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
# Same as slam.launch but with log level ERROR to suppress TF_OLD_DATA warnings
# when use_fake_odom (static tf + clock_publisher).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    slam_params_file = PathJoinSubstitution([
        FindPackageShare("rtk2026_slam"), "config", "slam_toolbox_params.yaml"
    ])
    slam_params_file_param = ParameterFile(slam_params_file, allow_substs=True)

    start_async_slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params_file_param,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        start_async_slam_toolbox_node,
        configure_event,
        activate_event,
    ])
