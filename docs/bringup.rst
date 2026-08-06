Сценарии запуска
================

``rtk2026_bringup`` связывает пакеты, но не владеет их параметрами:

* Serial-конфигурация остаётся в ``rtk2026_driver``;
* параметры SLAM остаются в ``rtk2026_slam``;
* URDF и контроллеры остаются в ``rtk2026_description``.

``arduino_launch.py``
---------------------

Находит ``rtk2026_driver/config/arduino_bridge.yaml`` через ament index и
запускает executable ``arduino_bridge`` с именем ноды ``arduino_bridge``.
Совпадение имени ноды с верхним YAML-ключом обязательно для применения
параметров.

``lidar_launch.py``
-------------------

Включает официальный ``sllidar_c1_launch.py`` со значениями:

.. list-table:: RPLIDAR C1
   :header-rows: 1

   * - Аргумент
     - Значение
   * - ``channel_type``
     - ``serial``
   * - ``serial_port``
     - ``/dev/rplidar``
   * - ``serial_baudrate``
     - 460800
   * - ``frame_id``
     - ``lidar_frame``
   * - ``inverted``
     - false
   * - ``angle_compensate``
     - true
   * - ``scan_mode``
     - ``Standard``

``slam_launch.py``
------------------

Общий адаптер для реального робота и симуляции. Он объявляет только
``use_sim_time``, находит конфигурацию пакета ``rtk2026_slam`` и включает
``slam_toolbox/online_async_launch.py`` с ``autostart=true``.

``real_slam.py``
----------------

Композиция четырёх launch-файлов:

.. code-block:: text

   display.launch.py  ──> robot_state_publisher, fixed TF
   arduino_launch.py  ──> /cmd_vel -> Arduino -> /odom + odom TF
   lidar_launch.py    ──> /scan
   slam_launch.py     ──> map, map -> odom

Для реального запуска: ``use_sim_time=false``, mesh отключены, webcam frame
включён.

``sim_slam_launch.py``
----------------------

Полный сценарий Gazebo SLAM:

1. разворачивает ``rtk2026_diff_drive_sim.urdf.xacro``;
2. запускает Gazebo Harmonic server-only;
3. публикует ``robot_description`` и статические TF;
4. создаёт модель ``rtk2026`` с начальным Z=0.03 м;
5. мостит ``/clock`` и ``/scan`` из Gazebo;
6. через 3 секунды запускает ``joint_state_broadcaster`` и
   ``diff_drive_controller``;
7. remap-ит вход/выход контроллера в общий API ``/cmd_vel`` и ``/odom``;
8. запускает ``slam_toolbox`` с симуляционным временем;
9. при ``use_rviz=true`` запускает RViz.

.. list-table:: Аргументы sim_slam_launch.py
   :header-rows: 1
   :widths: 28 26 46

   * - Аргумент
     - Default
     - Назначение
   * - ``world``
     - ``rtk2026_slam_world.sdf``
     - Абсолютный путь к Gazebo world; сюда передаётся и
       ``polygon_5x5.world``.
   * - ``use_meshes``
     - false
     - STL visual вместо примитивов.
   * - ``use_rviz``
     - true
     - Окно RViz в X11/noVNC.
   * - ``rviz_config``
     - ``rtk2026_sim_slam.rviz``
     - Пользовательская конфигурация RViz.

Запуск произвольного мира:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/absolute/path/to/polygon_5x5.world

Контроллеры запускаются ``spawner`` с timeout 60 секунд. ``TimerAction(3.0)``
только уменьшает стартовую гонку; фактическая синхронизация обеспечивается
ожиданием сервисов ``/controller_manager``.

Исходники: `launch/ <https://github.com/Hanqnero/RTK2026/tree/main/src/rtk2026_bringup/launch>`_.
