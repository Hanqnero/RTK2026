#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = Path(get_package_share_directory("rtk2026_route_nav"))
    params_file = pkg / "config" / "nav2_route_params.yaml"
    graph_file = pkg / "config" / "graph.geojson"
    lifecycle_params = pkg / "config" / "route_lifecycle_manager.yaml"

    return LaunchDescription(
        [
            Node(
                package="nav2_route",
                executable="route_server",
                name="route_server",
                output="screen",
                parameters=[
                    str(params_file),
                    {"graph_filepath": str(graph_file)},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                output="screen",
                parameters=[str(lifecycle_params)],
            ),
        ]
    )
