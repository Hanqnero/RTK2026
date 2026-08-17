Мир проверки SLAM
=================

``rtk2026_slam_world.sdf`` — небольшой диагностический мир 8×6 м. Он содержит
несимметричные ориентиры, чтобы scan matcher не путал геометрически одинаковые
участки.

Состав мира
-----------

.. list-table:: Статические объекты
   :header-rows: 1

   * - Имя
     - Геометрия/поза
     - Назначение
   * - ``ground_plane``
     - plane 20×20 м
     - Пол, friction μ=μ₂=1.
   * - ``north/south_wall``
     - 8.2×0.2×1.0 м, Y=±3
     - Горизонтальные границы.
   * - ``west/east_wall``
     - 0.2×6.2×1.0 м, X=±4
     - Вертикальные границы.
   * - ``long_partition``
     - box 2.2×0.18×0.8 м, yaw=0.35
     - Наклонный протяжённый ориентир.
   * - ``square_column``
     - box 0.55×0.55×1.0 м
     - Угловой ориентир.
   * - ``round_column``
     - cylinder radius 0.35 м
     - Отличимый круговой объект.

Physics использует шаг 1 мс и real-time factor 1.0. Мир подключает системы
Physics, UserCommands, SceneBroadcaster, Sensors с Ogre2 и отдельную систему
Imu. ``polygon_5x5.world`` содержит тот же IMU system plugin.

Другой world
------------

Launch принимает абсолютный путь:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/path/to/polygon_5x5.world

World должен содержать как минимум Physics, UserCommands, SceneBroadcaster,
Sensors и Imu plugins, иначе создание модели, lidar, IMU или GUI-данные могут
не работать.

Электролизный полигон
---------------------

``rtk2026_description/worlds/electrolysis_world_modular`` содержит платформу,
заднюю стену и
16 повторно используемых блоков электродов. Между левой и правой группами
оставлен центральный проход шириной 1 м. Модель ``aruco_marker`` добавляет
по центру внутренней стороны задней стены один стандартный маркер OpenCV ArUco
из словаря ``DICT_4X4_50`` с ``id=0``. Текстура воспроизводимо создаётся
``scripts/generate_aruco_marker.py`` через API OpenCV, а не задаётся вручную.
У маркера отсутствует collision: он нужен только RGB-D SLAM и не влияет на
проходимость или лидарную карту.

Для движения вдоль прохода робот ставится с yaw=π/2:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     slam_mode:=visual
