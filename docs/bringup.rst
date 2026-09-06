Сценарии запуска
================

``rtk2026_bringup`` связывает пакеты, но не владеет их параметрами:

* Serial-конфигурация остаётся в ``rtk2026_driver``;
* параметры SLAM остаются в ``rtk2026_slam``;
* параметры EKF и AMCL остаются в ``rtk2026_localization``;
* URDF и контроллеры остаются в ``rtk2026_description``.

``arduino_launch.py``
---------------------

Находит ``rtk2026_driver/config/arduino_bridge.yaml`` через ament index и
запускает executable ``arduino_bridge`` с именем ноды ``arduino_bridge``.
Совпадение имени ноды с верхним YAML-ключом обязательно для применения
параметров.

``lidar_launch.py``
-------------------

Запускает ``sllidar_node`` напрямую, чтобы перезапускать драйвер после
неудачного рукопожатия. Аргумент ``model`` выбирает профиль C1 или запасного
A1M8. Профиль C1 использует следующие значения:

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

``full.launch.py``
------------------

Полный аппаратный слой реального робота без алгоритмов:

.. code-block:: text

   display.launch.py  ──> robot_state_publisher, fixed TF
   arduino_launch.py  ──> /cmd_vel -> Arduino -> /wheel/odom
   lidar_launch.py    ──> /scan
   imu_launch.py      ──> /imu/data

По умолчанию запускаются все четыре части. Для стендовой отладки драйверы
можно отключать аргументами ``use_arduino``, ``use_lidar`` и ``use_imu``;
``lidar_model:=a1`` выбирает запасной A1M8 вместо штатного C1.

Launch намеренно не запускает EKF, SLAM/AMCL, Nav2, компьютерное зрение,
диагностику или RViz. Эти алгоритмы и инструменты выбираются и запускаются
отдельно. Драйвер USB-камеры также остаётся в отдельном compose-сервисе,
потому что у камеры свой device lifecycle; её фиксированный TF при этом
входит в публикуемое описание робота.

.. code-block:: bash

   ros2 launch rtk2026_bringup full.launch.py

``real_slam.py``
----------------

Композиция шести launch-файлов:

.. code-block:: text

   display.launch.py  ──> robot_state_publisher, fixed TF
   arduino_launch.py  ──> /cmd_vel -> Arduino -> /wheel/odom
   lidar_launch.py    ──> /scan
   imu_launch.py      ──> /imu/data
   ekf.launch.py      ──> /odometry/filtered + odom -> base_footprint
   slam_launch.py     ──> map, map -> odom

Для реального запуска: ``use_sim_time=false``; геометрия и camera frame
включены в латченный ``/robot_description`` для удалённого RViz.
``ekf_real.yaml`` объединяет ``vx``, ограничение ``vy=0`` из
энкодеров и ``angular_velocity.z`` BMI270. Нода BMI270 подключается к I²C
Raspberry Pi; её запускает ``imu_launch.py`` и она публикует ``/imu/data`` в
``imu_link``. Аргумент ``use_rviz`` по умолчанию false, а ``ekf_config``
позволяет явно выбрать другой YAML.

``real_localization.py``
------------------------

Аппаратная часть и EKF совпадают с ``real_slam.py``, но глобальную
трансформацию ``map -> odom`` публикует AMCL:

.. code-block:: text

   display + Arduino + RPLIDAR + Pi BMI270 + EKF
                                  │
   map_server ── /map ────────────┼──> AMCL ──> map -> odom
   RPLIDAR ───── /scan ───────────┘

Обязательный аргумент ``map`` — абсолютный путь к YAML карты. Одновременный
запуск этого сценария и ``real_slam.py`` недопустим.

``sim_slam_launch.py``
----------------------

Сценарий Gazebo с локальным EKF и выбираемой глобальной локализацией:

1. выбирает одну модель: ``tracked`` или ``diff_drive``;
2. один раз запускает Gazebo Harmonic, добавляя ``-s`` только в headless-режиме;
3. публикует ``robot_description`` и статические TF выбранной модели;
4. создаёт физическую модель в заданной начальной позе;
5. загружает общие связи Gazebo→ROS из ``config/gazebo_bridge.yaml``;
6. только для ``diff_drive`` запускает ``joint_state_broadcaster`` и
   ``diff_drive_controller``;
7. запускает EKF, который публикует ``/odometry/filtered`` и odom TF;
8. при ``slam_mode=lidar`` запускает ``slam_toolbox``, а при
   ``slam_mode=visual`` — RTAB-Map RGB-D;
9. при ``use_rviz=true`` запускает RViz.

.. list-table:: Аргументы sim_slam_launch.py
   :header-rows: 1
   :widths: 28 26 46

   * - Аргумент
     - Default
     - Назначение
   * - ``robot_model``
     - tracked
     - Ходовая часть: ``tracked`` или ``diff_drive``.
   * - ``world``
     - ``electrolysis.sdf``
     - Абсолютный путь к Gazebo world; сюда передаётся и
       ``polygon_5x5.world``.
   * - ``use_meshes``
     - false
     - STL visual вместо примитивов.
   * - ``use_gazebo_gui``
     - false
     - Запустить клиент Gazebo вместе с сервером; окно выводится в noVNC.
   * - ``use_rviz``
     - true
     - Окно RViz в X11/noVNC.
   * - ``slam_mode``
     - visual
     - ``lidar`` — ``slam_toolbox``;
       ``visual`` — RTAB-Map RGB-D;
       ``none`` — базовый стек без глобальной локализации.
   * - ``rtabmap_database``
     - ``/workspace/records/rtabmap/rtk2026.db``
     - База графа и данных визуального SLAM.
   * - ``rtabmap_localization``
     - false
     - ``false`` строит карту, ``true`` локализуется по существующей базе.
   * - ``rtabmap_args``
     - пусто
     - Дополнительные аргументы; ``--delete_db_on_start`` создаёт новую базу.
   * - ``rviz_config``
     - ``rtk2026_sim_slam.rviz``
     - Пользовательская конфигурация RViz.
   * - ``spawn_x`` / ``spawn_y`` / ``spawn_z`` / ``spawn_yaw``
     - -0.3 / -0.1 / 0.081 / π/2
     - Начальная поза модели; Z задаётся относительно поверхности world,
       yaw — в радианах.

Запуск произвольного мира и модели
----------------------------------

Команды запуска собраны в :doc:`running`; здесь - что при этом происходит.

Для ``robot_model:=diff_drive`` контроллеры запускаются ``spawner`` с
timeout 60 секунд. ``TimerAction(3.0)`` только уменьшает стартовую гонку;
фактическая синхронизация обеспечивается ожиданием сервисов
``/controller_manager``. Для ``tracked`` эти процессы вообще не создаются.

Runtime-блоки симуляционного launch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: Ноды, которые остаются после запуска
   :header-rows: 1
   :widths: 27 30 43

   * - Нода
     - Главные интерфейсы
     - Откуда появляется
   * - ``/robot_state_publisher``
     - ``/robot_description``, ``/tf`` и ``/tf_static``
     - Один из двух условных ``Node`` для выбранного Xacro.
   * - ``/gazebo_bridge``
     - Gazebo→ROS: часы, лидар, IMU, RGB-D и ground truth
     - Штатный ``parameter_bridge`` с ``config/gazebo_bridge.yaml``.
   * - ``/tracked_drive_bridge``
     - ROS→Gazebo ``/cmd_vel``; Gazebo→ROS ``/wheel/odom``
     - Только для ``robot_model:=tracked``.
   * - ``/controller_manager``
     - control services, ``/robot_description``
     - Только ``diff_drive``: плагин ``gz_ros2_control``.
   * - ``/joint_state_broadcaster``
     - Publisher ``/joint_states``
     - Загружается краткоживущим ``spawner``.
   * - ``/diff_drive_controller``
     - Subscriber ``/cmd_vel``; publishers ``/wheel/odom`` и ``cmd_vel_out``
     - Только ``diff_drive``: загружается вторым ``spawner``.
   * - ``/ekf_filter_node``
     - Subscribers ``/wheel/odom`` и ``/imu/data``;
       publishers ``/odometry/filtered`` и ``/tf``
     - Включён через ``rtk2026_localization/ekf.launch.py``.
   * - ``/slam_toolbox``
     - Subscriber ``/scan``; publishers ``/map``, ``/tf`` и lifecycle events
     - Включён через ``slam_launch.py`` только при ``slam_mode=lidar``.
   * - ``/rtabmap/rtabmap``
     - Subscribers RGB, aligned depth и ``/odometry/filtered``;
       publishers ``/map`` и ``map -> odom``
     - Включён только при ``slam_mode=visual``.
   * - ``/rviz2``
     - ``/map``, ``/scan``, ``/tf``, ``/tf_static`` и другие display topics
     - Запускается только при ``use_rviz=true``.

``spawn_rtk2026_diff_drive`` или ``spawn_rtk2026_tracked`` выполняет одно
действие и завершается, поэтому поздний ``rqt_graph`` его не показывает.
Два ``spawner`` появляются только у колёсной модели.

Практические команды для каждого launch и проверка получившегося графа:
:doc:`running`.

Исходники: `launch/ <https://github.com/Hanqnero/RTK2026/tree/main/src/rtk2026_bringup/launch>`_.
