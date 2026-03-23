# Gazebo + diff_robot + RViz + TurtleBot3 Autorace (lane, sign, obstacle). Optional SLAM for mapping.
# Use with: -World autorace -Autorace. Add -Map to build map (slam_toolbox) alongside lane/sign/obstacle.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _get_robot_urdf_path() -> str:
    default = "/workspace/src/diff_robot/urdf/diff_robot.urdf"
    path = default
    try:
        with open("/tmp/robot_urdf_path", "r") as f:
            path = (f.read().strip() or default)
    except Exception:
        path = os.environ.get("ROBOT_URDF_PATH", default)
    if path != default and not os.path.isfile(path):
        path = default
    return path


def _robot_description_string() -> str:
    try:
        with open(_get_robot_urdf_path(), "r") as f:
            return f.read()
    except Exception:
        return ""


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    R = LaunchConfiguration("R")
    P = LaunchConfiguration("P")
    Y = LaunchConfiguration("Y")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_delay = LaunchConfiguration("spawn_delay")
    autorace_delay = LaunchConfiguration("autorace_delay")
    use_slam = LaunchConfiguration("use_slam", default="false")
    lane_calibration_mode = LaunchConfiguration("lane_calibration_mode", default="false")
    use_camera_calibration = LaunchConfiguration("use_camera_calibration", default="false")
    lane_image_source = LaunchConfiguration("lane_image_source", default="ipm")

    _robot_urdf_path = _get_robot_urdf_path()
    robot_description = {"robot_description": _robot_description_string()}
    rviz_config_default = "/workspace/src/diff_robot/urdf/rviz_autorace.rviz"
    rviz_config = LaunchConfiguration("rvizconfig")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    _gazebo_plugin_dir = "/workspace/build/turtlebot3_gazebo"
    _env_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    _gazebo_models_dirs = [
        "/workspace/src/turtlebot3_gazebo/models",
        "/turtlebot3_simulations/turtlebot3_gazebo/models",
    ]
    _plugin_path = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    _model_path = os.environ.get("GAZEBO_MODEL_PATH", "") if not _env_model_path else ""
    _model_path_str = _env_model_path or (":".join(_gazebo_models_dirs) + (":" + _model_path if _model_path else ""))
    _gazebo_env = {
        "GAZEBO_PLUGIN_PATH": _gazebo_plugin_dir + (":" + _plugin_path if _plugin_path else ""),
        "GAZEBO_MODEL_PATH": _model_path_str,
    }

    gazebo_launch = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", world],
        output="screen",
        additional_env=_gazebo_env,
    )

    spawn_entity = TimerAction(
        period=spawn_delay,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file", _robot_urdf_path, "-entity", "diff_robot", "-timeout", "120.0",
                    "-x", x, "-y", y, "-z", z, "-R", R, "-P", P, "-Y", Y,
                ],
                output="screen",
            ),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    rviz_delayed = TimerAction(period=2.0, actions=[rviz_node])

    autorace_launch_path = PathJoinSubstitution([
        FindPackageShare("rtk2026_simulation"),
        "launch",
        "rtk2026_autorace.launch.py",
    ])
    autorace_group = TimerAction(
        period=autorace_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(autorace_launch_path),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "lane_calibration_mode": lane_calibration_mode,
                    "use_camera_calibration": use_camera_calibration,
                    "lane_image_source": lane_image_source,
                }.items(),
            ),
        ],
    )

    slam_params = PathJoinSubstitution([
        FindPackageShare("rtk2026_nav2_explorer"),
        "config",
        "slam_toolbox_autorace.yaml",
    ])
    slam_group = GroupAction(
        condition=IfCondition(PythonExpression(["'", use_slam, "' == 'true'"])),
        actions=[
            TimerAction(
                period=autorace_delay,
                actions=[
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[slam_params, {"use_sim_time": use_sim_time}],
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="/workspace/src/diff_robot/world/silverstone_track.world"),
        DeclareLaunchArgument("x", default_value="-5.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.15"),
        DeclareLaunchArgument("R", default_value="0.0"),
        DeclareLaunchArgument("P", default_value="0.0"),
        DeclareLaunchArgument("Y", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rvizconfig", default_value=rviz_config_default),
        DeclareLaunchArgument("spawn_delay", default_value="50.0", description="Wait before spawn (autorace world is heavy)."),
        DeclareLaunchArgument("autorace_delay", default_value="58.0", description="Wait before starting autorace stack after spawn."),
        DeclareLaunchArgument("use_slam", default_value="true", description="Run slam_toolbox to build /map for RViz (script passes by default; use -NoMap to disable)."),
        DeclareLaunchArgument("lane_calibration_mode", default_value="false", description="Run detect_lane in calibration mode (tune lane params in rqt, save lane.yaml)."),
        DeclareLaunchArgument("use_camera_calibration", default_value="false", description="Use turtlebot3_autorace_camera for /camera/image_projected display."),
        DeclareLaunchArgument("lane_image_source", default_value="ipm", description="Which bird's-eye topic to feed detect_lane: ipm or camera."),
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        rviz_delayed,
        autorace_group,
        slam_group,
    ])
