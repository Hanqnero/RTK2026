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
Physics, UserCommands, SceneBroadcaster и Sensors с Ogre2.

Другой world
------------

Launch принимает абсолютный путь:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/path/to/polygon_5x5.world

World должен содержать как минимум Physics, UserCommands, SceneBroadcaster и
Sensors plugins, иначе создание модели, lidar или GUI-данные могут не работать.
