# Launch TurtleBot3 Autorace pipeline: lane keeping via PD regulator, sign detection, obstacle avoidance.
# Expects: Gazebo + diff_robot already running with /camera/image_raw, /scan, /odom.
# Publishes: /cmd_vel (detect_lane → /control/lane → control_lane PD+avoid mux → /cmd_vel).
# lane_calibration_mode:=true -- detect_lane in calibration mode (tune in rqt, save lane.yaml).
# use_camera_calibration:=true -- run turtlebot3_autorace_camera (intrinsic+extrinsic) for view (/camera/image_projected).
# lane_image_source:=ipm|camera -- which bird's-eye topic to feed detect_lane.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    mission = LaunchConfiguration("mission", default="construction")
    lane_calibration_mode = LaunchConfiguration("lane_calibration_mode", default="false")
    use_camera_calibration = LaunchConfiguration("use_camera_calibration", default="false")
    lane_image_source = LaunchConfiguration("lane_image_source", default="ipm")

    ipm_projected_topic = "/rtk_autorace_ipm/image_projected"
    ipm_compensated_topic = "/rtk_autorace_ipm/image_compensated"

    try:
        detect_share = get_package_share_directory("turtlebot3_autorace_detect")
        lane_param = os.path.join(detect_share, "param", "lane", "lane.yaml")
    except Exception:
        lane_param = None

    try:
        camera_share = get_package_share_directory("turtlebot3_autorace_camera")
        intrinsic_launch = os.path.join(camera_share, "launch", "intrinsic_camera_calibration.launch.py")
        extrinsic_launch = os.path.join(camera_share, "launch", "extrinsic_camera_calibration.launch.py")
    except Exception:
        camera_share = None
        intrinsic_launch = extrinsic_launch = None

    # IPM relay node: /camera/image_raw → bird's-eye projected/compensated
    relay_node = Node(
        package="rtk2026_peripherals",
        executable="image_relay_autorace",
        name="image_relay_autorace",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"use_ipm": True},
            # camera_joint z=0.038 + wheel_radius=0.01 = 0.048m above ground
            {"camera_height_m": 0.048},
            # camera_joint rpy="0.0 0.3490 0.0" => 20deg downward pitch
            {"camera_pitch_rad": 0.3490},
            # with 20deg pitch road visible from ~5cm to ~40cm ahead
            {"y_near_m": 0.05},
            {"y_far_m": 0.40},
            # lane width ~0.35m, ±0.22m gives full lane + margin
            {"x_left_m": -0.22},
            {"x_right_m": 0.22},
            {"ipm_vertical_flip": False},
            {"projected_topic": ipm_projected_topic},
            {"compensated_topic": ipm_compensated_topic},
        ],
    )

    # max_vel publisher (used by avoid_construction to cap speed)
    max_vel_node = Node(
        package="rtk2026_peripherals",
        executable="autorace_max_vel_publisher",
        name="autorace_max_vel_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"max_vel": 0.032},
            {"publish_interval_sec": 0.5},
        ],
    )

    detect_lane_common_params = (
        [
            {"use_sim_time": use_sim_time, "is_detection_calibration_mode": lane_calibration_mode},
            lane_param,
        ]
        if lane_param
        else [{"use_sim_time": use_sim_time, "is_detection_calibration_mode": lane_calibration_mode}]
    )

    detect_lane_node_ipm = Node(
        package="turtlebot3_autorace_detect",
        executable="detect_lane",
        name="detect_lane",
        output="screen",
        parameters=detect_lane_common_params,
        remappings=[
            ("/detect/image_input", ipm_projected_topic),
            ("/detect/image_input/compressed", ipm_projected_topic + "/compressed"),
            ("/detect/lane", "/control/lane"),
        ],
        condition=IfCondition(PythonExpression(["'", lane_image_source, "' != 'camera'"])),
    )

    detect_lane_node_camera = Node(
        package="turtlebot3_autorace_detect",
        executable="detect_lane",
        name="detect_lane",
        output="screen",
        parameters=detect_lane_common_params,
        remappings=[
            ("/detect/image_input", "/camera/image_projected"),
            ("/detect/image_input/compressed", "/camera/image_projected/compressed"),
            ("/detect/lane", "/control/lane"),
        ],
        condition=IfCondition(PythonExpression(["'", lane_image_source, "' == 'camera'"])),
    )

    # control_lane: PD regulator + avoid mux, subscribes /control/lane → publishes /cmd_vel
    control_lane_node = Node(
        package="turtlebot3_autorace_mission",
        executable="control_lane",
        name="control_lane",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/control/cmd_vel", "/cmd_vel")],
        condition=UnlessCondition(lane_calibration_mode),
    )

    # avoid_construction: lidar-based obstacle avoidance, publishes /avoid_control
    avoid_construction_node = Node(
        package="turtlebot3_autorace_mission",
        executable="avoid_construction",
        name="avoid_construction",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"danger_distance": 0.12},
            {"danger_width": 0.06},
            {"speed": 0.02},
            {"robot_width_m": 0.05},
            {"robot_height_m": 0.08},
        ],
        remappings=[
            ("/camera/image_projected", ipm_projected_topic),
        ],
        condition=UnlessCondition(lane_calibration_mode),
    )

    # detect_construction_sign for sign detection
    detect_sign_node = Node(
        package="turtlebot3_autorace_detect",
        executable="detect_construction_sign",
        name="detect_construction_sign",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/detect/image_input", ipm_compensated_topic),
            ("/detect/image_input/compressed", ipm_compensated_topic + "/compressed"),
        ],
    )

    actions = [
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use sim time"),
        DeclareLaunchArgument(
            "lane_calibration_mode",
            default_value="false",
            description="Run detect_lane in calibration mode to tune lane params in rqt and save lane.yaml",
        ),
        DeclareLaunchArgument(
            "use_camera_calibration",
            default_value="false",
            description="Use turtlebot3_autorace_camera (intrinsic+extrinsic) for /camera/image_projected.",
        ),
        DeclareLaunchArgument(
            "lane_image_source",
            default_value="ipm",
            description="Which topic to feed detect_lane: ipm or camera.",
        ),
        DeclareLaunchArgument(
            "mission",
            default_value="construction",
            description="Mission type for sign detection",
        ),
        max_vel_node,
        relay_node,
    ]

    if intrinsic_launch and os.path.isfile(intrinsic_launch):
        actions.append(
            GroupAction(
                condition=IfCondition(use_camera_calibration),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(intrinsic_launch),
                        launch_arguments={"use_sim_time": use_sim_time}.items(),
                    ),
                ],
            ),
        )
    if extrinsic_launch and os.path.isfile(extrinsic_launch):
        actions.append(
            GroupAction(
                condition=IfCondition(use_camera_calibration),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(extrinsic_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "calibration_mode": lane_calibration_mode,
                        }.items(),
                    ),
                ],
            ),
        )

    actions.extend([
        detect_lane_node_ipm,
        detect_lane_node_camera,
        control_lane_node,
        # Delay avoid_construction 15s: lane_state=1 at startup triggers false avoidance otherwise
        TimerAction(period=15.0, actions=[avoid_construction_node]),
        detect_sign_node,
    ])

    return LaunchDescription(actions)
