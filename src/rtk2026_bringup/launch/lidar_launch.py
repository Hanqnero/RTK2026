"""Запуск драйвера RPLIDAR с параметрами реального подключения.

Модель выбирается аргументом ``model``, по умолчанию C1 - тот, что стоит
на роботе. Профиль A1M8 сохранён рабочим: он остаётся запасным лидаром,
и его параметры добыты замерами, которые не хочется восстанавливать
заново.

    ros2 launch rtk2026_bringup lidar_launch.py             # C1
    ros2 launch rtk2026_bringup lidar_launch.py model:=a1   # A1M8
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ~ Профили лидаров.
#
# Различаются скоростью порта и режимом сканирования, и оба параметра
# обязательны: с чужой скоростью драйвер не поднимется вовсе, а с чужим
# именем режима не сможет запустить сканирование.
LIDAR_PROFILES = {
    "c1": {
        # Штатная скорость C1. У A1M8 она 115200, и значения не
        # взаимозаменяемы.
        "serial_baudrate": 460800,

        # ~ Режим сканирования C1.
        #
        # Standard выбран как режим, существующий у всех RPLIDAR, а не как
        # заведомо лучший для C1: фактический список режимов этого
        # экземпляра здесь не проверялся.
        #
        # Драйвер печатает доступные режимы при старте (он опрашивает
        # GET_LIDAR_CONF). После первого запуска стоит посмотреть
        #
        #     docker logs -f rtk2026-lidar
        #
        # и, если C1 предлагает режим с большей выборкой, поставить его
        # здесь - как это сделано для A1M8 ниже.
        #
        # Пустое значение оставлять нельзя: драйвер тогда берёт "типичный"
        # режим через отдельный путь SDK, а он отрабатывает менее надёжно,
        # чем выбор по имени.
        "scan_mode": "Standard",
    },
    "a1": {
        "serial_baudrate": 115200,

        # ~ Режим сканирования A1M8.
        #
        # Опрошенные у самого лидара режимы (GET_LIDAR_CONF), все
        # с дальностью 12 м:
        #
        #     0  Standard     2.0 кГц
        #     1  Express      3.9 кГц
        #     2  Boost        7.9 кГц
        #     3  Sensitivity  7.9 кГц   <- типичный по мнению лидара
        #     4  Stability    5.0 кГц
        #
        # Sensitivity выбран как максимальная выборка при лучшей
        # чувствительности к слабо отражающим поверхностям.
        # Замерено на роботе: 1082 луча за оборот при 6.99 Гц.
        "scan_mode": "Sensitivity",
    },
}


def _lidar_node(context, *args, **kwargs) -> list[Node]:
    """Собрать ноду драйвера по выбранной модели.

    Профиль выбирается здесь, а не подстановкой в параметрах, потому что
    ``serial_baudrate`` должен уйти в ноду целым числом. Подстановка
    отдала бы строку, и драйвер отверг бы параметр по типу.
    """

    model = LaunchConfiguration("model").perform(context).lower()

    profile = LIDAR_PROFILES.get(model)
    if profile is None:
        known = ", ".join(sorted(LIDAR_PROFILES))
        raise RuntimeError(
            f"неизвестная модель лидара {model!r}; известны: {known}"
        )

    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",

        # ~ Перезапуск до успешного старта.
        #
        # Рукопожатие проходит не с первой попытки: драйвер шлёт GET_HEALTH
        # с коротким таймаутом и при отсутствии ответа падает с
        # SL_RESULT_OPERATION_TIMEOUT и кодом 255. Замерено на A1M8:
        # успешный старт приходится на 3-4 попытку, и мешает раскрутка
        # мотора в момент рукопожатия - лидар исправен и на остановленном
        # моторе отвечает 10 раз из 10. Повтор через две секунды попадает
        # в паузу, когда обороты уже установились.
        #
        # Для C1 это не перемерялось; respawn оставлен, потому что от
        # успешного старта он не отнимает ничего.
        respawn=True,
        respawn_delay=2.0,

        parameters=[
            {
                # Используем serial-соединение.
                "channel_type": "serial",

                # Путь к лидару внутри Docker. Пробрасывается в
                # pi/docker/docker-compose.pi.yml по идентификатору
                # устройства, а не по номеру ttyUSB.
                "serial_port": "/dev/rplidar",

                "serial_baudrate": profile["serial_baudrate"],
                "scan_mode": profile["scan_mode"],

                # Должно совпадать с link в URDF.
                "frame_id": "lidar_frame",

                # Лидар установлен обычной стороной вверх.
                "inverted": False,

                # Включаем угловую компенсацию драйвера.
                "angle_compensate": True,

                # Частоту вращения драйвер параметром не принимает.
                # За один оборот при 0.25 м/с робот проезжает несколько
                # сантиметров, а slam_toolbox скан не расшивает - отсюда
                # рекомендация картографировать на пониженной скорости.
            }
        ],
    )

    return [lidar_node]


def generate_launch_description() -> LaunchDescription:
    """Поднять ``sllidar_node`` с перезапуском до успешного старта.

    Нода объявлена здесь напрямую, а не через launch вендора. Причина
    одна: нужен ``respawn``, а включённый чужой launch его не отдаёт.
    """

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="c1",
                choices=sorted(LIDAR_PROFILES),
                description="Модель лидара: c1 на роботе, a1 - запасной A1M8.",
            ),
            OpaqueFunction(function=_lidar_node),
        ]
    )
