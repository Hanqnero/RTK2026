#!/usr/bin/env python3
"""Nav2 VectorObjectServer: keepout-полигоны как costmap filter mask.

Поднимает штатные бинарники Nav2 (``vector_object_server``,
``costmap_filter_info_server``) плюс ``keepout_click_tool`` - ноду этого
пакета, которая копит клики в RViz (``/clicked_point``) и коммитит их
как полигон через сервис ``/vector_object_server/add_shapes``.

Самодостаточный launch-файл: не поднимает ни карту, ни costmap, ни граф
дорог. Раньше все три поднимались одним монолитным launch-файлом в
``rtk2026_route_nav``; здесь - только то, что относится к vector objects,
чтобы модуль можно было включать (``IncludeLaunchDescription``) независимо
от конкретного сценария навигации.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    pkg = Path(get_package_share_directory("rtk2026_vector_objects"))
    default_params = pkg / "config" / "vector_object_server_params.yaml"

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    add_shapes_service = LaunchConfiguration("add_shapes_service")
    frame_id = LaunchConfiguration("frame_id")
    zones_path = LaunchConfiguration("zones_path")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Использовать /clock вместо системного времени.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="YAML параметров vector_object_server и costmap_filter_info_server.",
            ),
            DeclareLaunchArgument(
                "add_shapes_service",
                default_value="/vector_object_server/add_shapes",
                description="Сервис добавления полигонов, который дёргает keepout_click_tool.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="map",
                description="Frame, в котором коммитятся полигоны по умолчанию.",
            ),
            DeclareLaunchArgument(
                "zones_path",
                default_value="",
                description=(
                    "Файл размеченных зон. Пусто — зоны живут до перезапуска. "
                    "Зоны привязаны к системе координат карты, поэтому файл "
                    "хранят рядом с ней."
                ),
            ),
            Node(
                package="nav2_map_server",
                executable="vector_object_server",
                name="vector_object_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_map_server",
                executable="costmap_filter_info_server",
                name="costmap_filter_info_server",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_vector",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": ["vector_object_server", "costmap_filter_info_server"]},
                ],
            ),
            Node(
                package="rtk2026_vector_objects",
                executable="keepout_click_tool",
                name="keepout_click_tool",
                output="screen",
                parameters=[
                    {"add_shapes_service": add_shapes_service},
                    {"frame_id": frame_id},
                    {"zones_path": zones_path},
                ],
            ),
        ]
    )
