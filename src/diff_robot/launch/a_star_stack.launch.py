import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Layout inside Docker image: package checked out as /workspace/src/diff_robot
    pkg_share = "/workspace/src/diff_robot"
    map_yaml = os.path.join(pkg_share, "map", "my_map.yaml")
    a_star_script = os.path.join(pkg_share, "planning", "a_star_path_finding.py")
    mppi_script = os.path.join(pkg_share, "control", "pure_pursit.py")

    return LaunchDescription(
        [
            # Map server on pre-built map (Nav2)
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_yaml, "use_sim_time": True}],
            ),
            # Bring map_server lifecycle node to active state
            Node(
                package="nav2_util",
                executable="lifecycle_bringup",
                name="map_server_lifecycle_bringup",
                output="screen",
                arguments=["map_server"],
            ),
            # AMCL localization against the static map
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            # Bring amcl lifecycle node to active state
            Node(
                package="nav2_util",
                executable="lifecycle_bringup",
                name="amcl_lifecycle_bringup",
                output="screen",
                arguments=["amcl"],
            ),
            # A* global planner: subscribes to /map, /goal_pose, /amcl_pose and publishes /planned_path
            ExecuteProcess(
                cmd=["python3", a_star_script],
                output="screen",
            ),
            # MPPI path follower: subscribes to /planned_path and /amcl_pose, publishes /cmd_vel
            ExecuteProcess(
                cmd=["python3", mppi_script],
                output="screen",
            ),
        ]
    )

