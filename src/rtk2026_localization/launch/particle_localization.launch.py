"""Запуск локализации частицами по заранее сохранённой карте.

Launch создаёт lifecycle-ноды ``map_server`` и ``amcl``, затем активирует их
через отдельный ``lifecycle_manager_localization``. Локальный EKF сюда
намеренно не включён: источник ``odom -> base_footprint`` должен быть запущен
отдельно и проверен до старта глобальной локализации.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Запустить Map Server, Nav2 AMCL и их lifecycle manager.

    Обязательный аргумент ``map`` принимает абсолютный путь к YAML карты.
    AMCL публикует ``map -> odom`` только после получения карты, сканов,
    локального TF и начальной позы.
    """

    # ~ Аргументы режима известной карты.
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    # ~ Конфигурация модели движения и лазерного измерения AMCL.
    amcl_config = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_localization"),
            "config",
            "amcl.yaml",
        ]
    )

    # ~ Публикация OccupancyGrid из пары YAML + PGM.
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ~ Глобальная коррекция map -> odom методом частиц.
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # ~ Configure и activate обеих lifecycle-нод в правильном порядке.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": [
                    "map_server",
                    "amcl",
                ],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Абсолютный путь к YAML заранее сохранённой карты.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Использовать часы из /clock.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Автоматически configure/activate Map Server и AMCL.",
            ),
            map_server,
            amcl,
            lifecycle_manager,
        ]
    )
