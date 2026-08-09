#!/usr/bin/env python3
"""Запуск стека Nav2.

Четыре сервера и менеджер жизненного цикла. По присланным позам ездит сам
Nav2 действием ``navigate_through_poses``, поэтому своей ноды-исполнителя
здесь нет.

Запретные зоны
--------------

Аргумент ``use_keepout`` включает в костмапах фильтр
``nav2_costmap_2d::KeepoutFilter``. Маску для него публикует
``vector_object_server`` из ``rtk2026_vector_objects``, который поднимается
своим лаунчем. Без него фильтр включать нельзя: он будет ждать несуществующую
информацию о фильтре и писать об этом в лог, ничего не запрещая.

Менеджер жизненного цикла
-------------------------

Поднимается с задержкой. Пока робот не заспавнен, нет ``joint_states``
и ``odom``, а значит разорвано TF ``odom -> base_footprint``:
``controller_server`` падает на активации, а ``bt_navigator`` остаётся
inactive. Задержка ждёт появления TF, поэтому её величина зависит от способа
запуска робота и задаётся аргументом.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

#: Серверы, которыми управляет менеджер жизненного цикла. Порядок значим:
#: bt_navigator активируется последним, когда всё, что он вызывает, готово.
LIFECYCLE_NODES = (
    ("nav2_planner", "planner_server"),
    ("nav2_controller", "controller_server"),
    ("nav2_behaviors", "behavior_server"),
    ("nav2_bt_navigator", "bt_navigator"),
)

#: Фильтр запретных зон, объявленный в config/nav2_params.yaml.
KEEPOUT_FILTER = "keepout_filter"


def _params(context) -> dict:
    """Собрать параметры стека с учётом аргументов.

    Список фильтров решается здесь, а не в YAML, потому что зависит от
    ``use_keepout``, а этот аргумент известен только в момент запуска.
    Перезаписывается он у обеих костмап сразу: срезать угол через запретную
    зону контроллер может ровно так же, как планировщик проложить путь.
    """
    use_keepout = LaunchConfiguration("use_keepout").perform(context)
    filters = [KEEPOUT_FILTER] if use_keepout.lower() in ("true", "1") else []

    return {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "default_nav_through_poses_bt_xml": LaunchConfiguration("through_poses_bt"),
        "default_nav_to_pose_bt_xml": LaunchConfiguration("to_pose_bt"),
        "filters": str(filters),
    }


def _stack(context, *_args, **_kwargs) -> list:
    """Серверы и менеджер. Собираются после разбора аргументов."""
    # Время и пути к деревьям перезаписываются на всех уровнях, включая
    # вложенные костмапы: подстановка отдельным словарём параметров до них
    # не доходит, потому что костмапа — вложенная нода со своим ключом.
    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        root_key="",
        param_rewrites=_params(context),
        convert_types=True,
    )

    servers = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[configured_params],
        )
        for package, executable in LIFECYCLE_NODES
    ]

    manager = TimerAction(
        period=LaunchConfiguration("lifecycle_delay_s"),
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    configured_params,
                    {
                        "autostart": LaunchConfiguration("autostart"),
                        # Активация ждёт костмапы и TF, а не отвечает мгновенно.
                        "service_timeout": 60.0,
                        # Ноль отключает bond: серверы этого стека переживают
                        # короткие пропадания без перезапуска.
                        "bond_timeout": 0.0,
                    },
                ],
            )
        ],
    )

    return [*servers, manager]


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare("rtk2026_nav2")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Брать время из /clock. Для реального робота false.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [share, "config", "nav2_params.yaml"]
                ),
                description="Параметры стека Nav2.",
            ),
            DeclareLaunchArgument(
                "through_poses_bt",
                default_value=PathJoinSubstitution(
                    [
                        share,
                        "behavior_trees",
                        "navigate_through_poses_static.xml",
                    ]
                ),
                description="Дерево поведения для navigate_through_poses.",
            ),
            DeclareLaunchArgument(
                "to_pose_bt",
                # Для одиночной цели берём штатное дерево Nav2: своего смысла
                # у неё в городском движении нет, а прибивать путь
                # к дистрибутиву незачем.
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("nav2_bt_navigator"),
                        "behavior_trees",
                        "navigate_w_replanning_only_if_goal_is_updated.xml",
                    ]
                ),
                description="Дерево поведения для navigate_to_pose.",
            ),
            DeclareLaunchArgument(
                "use_keepout",
                default_value="false",
                description=(
                    "Включить фильтр запретных зон в костмапах. Требует "
                    "запущенного vector_object_server, иначе фильтр будет "
                    "ждать информацию о фильтре и ничего не запретит."
                ),
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Поднимать серверы в active сразу.",
            ),
            DeclareLaunchArgument(
                "lifecycle_delay_s",
                default_value="12.0",
                description=(
                    "Задержка перед активацией: ждём TF odom -> base_footprint. "
                    "В симуляции робот спавнится не сразу."
                ),
            ),
            OpaqueFunction(function=_stack),
        ]
    )
