"""Диагностика реального робота со SLAM Toolbox."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Подключить общую диагностику с профилем реального робота.

    Запускается на самой Raspberry Pi, а не на ноутбуке. Мониторы узлов
    опираются на ROS graph, а он через DDS Router не проходит достоверно:
    проверять состав системы надо там, где граф настоящий. Сами топики
    /diagnostics и /diagnostics_agg при этом маршрутизируются нормально,
    и смотреть их с ноутбука можно. Подробнее в docs/transport_check.rst.
    """

    use_gui = LaunchConfiguration("use_gui")
    records_path = LaunchConfiguration("records_path")

    common_launch = PathJoinSubstitution(
        [
            FindPackageShare("rtk2026_observability"),
            "launch",
            "diagnostics.launch.py",
        ]
    )

    return LaunchDescription(
        [
            # Графические инструменты по умолчанию выключены: дисплея
            # на роботе нет, а Qt-пакеты в образ Raspberry Pi не входят.
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument(
                "records_path",
                default_value="/workspace/records",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(common_launch),
                launch_arguments={
                    "profile": "real",
                    "use_gui": use_gui,
                    "localization_mode": "mapping",
                    "slam_backend": "lidar",
                    "records_path": records_path,
                }.items(),
            ),
        ]
    )
