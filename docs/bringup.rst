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

Композиция пяти launch-файлов:

.. code-block:: text

   display.launch.py  ──> robot_state_publisher, fixed TF
   arduino_launch.py  ──> /cmd_vel -> Arduino -> /wheel/odom
   lidar_launch.py    ──> /scan
   ekf.launch.py      ──> /odometry/filtered + odom -> base_footprint
   slam_launch.py     ──> map, map -> odom

Для реального запуска: ``use_sim_time=false``, mesh отключены, webcam frame
включён. ``ekf_real.yaml`` объединяет ``vx``, ограничение ``vy=0`` из
энкодеров и ``angular_velocity.z`` BMI270. Нода BMI270 подключается к I²C
Raspberry Pi и должна отдельно публиковать ``/imu/data`` в ``imu_link``;
текущий bringup её пока не запускает. Аргумент ``use_rviz`` по умолчанию
false, а ``ekf_config`` позволяет явно выбрать другой YAML.

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

1. разворачивает ``rtk2026_diff_drive_sim.urdf.xacro``;
2. запускает Gazebo Harmonic server-only;
3. публикует ``robot_description`` и статические TF;
4. создаёт модель ``rtk2026`` с начальным Z=0.03 м;
5. мостит ``/clock``, ``/scan``, ``/imu/data`` и диагностическую
   ``/ground_truth/odom`` из Gazebo;
6. через 3 секунды запускает ``joint_state_broadcaster`` и
   ``diff_drive_controller``;
7. remap-ит вход/выход контроллера в ``/cmd_vel`` и ``/wheel/odom``;
8. запускает EKF, который публикует ``/odometry/filtered`` и odom TF;
9. при ``use_slam=true`` запускает ``slam_toolbox``;
10. при ``use_rviz=true`` запускает RViz.

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
   * - ``use_slam``
     - true
     - ``slam_toolbox`` для mapping. Перед отдельным запуском AMCL задаётся
       ``false``.
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

Runtime-блоки симуляционного launch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table:: Ноды, которые остаются после запуска
   :header-rows: 1
   :widths: 27 30 43

   * - Нода
     - Главные интерфейсы
     - Откуда появляется
   * - ``/robot_state_publisher``
     - ``/joint_states``, ``/robot_description``, ``/tf``, ``/tf_static``
     - Явный ``Node`` в sim launch.
   * - ``/gazebo_bridge``
     - Gazebo→ROS ``/clock``, ``/scan`` и ``/imu/data``
     - Явный ``parameter_bridge``.
   * - ``/controller_manager``
     - control services, ``/robot_description``
     - Плагин ``gz_ros2_control`` внутри модели Gazebo.
   * - ``/joint_state_broadcaster``
     - Publisher ``/joint_states``
     - Загружается краткоживущим ``spawner``.
   * - ``/diff_drive_controller``
     - Subscriber ``/cmd_vel``; publishers ``/wheel/odom`` и ``cmd_vel_out``
     - Загружается вторым ``spawner`` и получает remap.
   * - ``/ekf_filter_node``
     - Subscriber ``/wheel/odom``; publishers ``/odometry/filtered`` и ``/tf``
     - Включён через ``rtk2026_localization/ekf.launch.py``.
   * - ``/slam_toolbox``
     - Subscriber ``/scan``; publishers ``/map``, ``/tf`` и lifecycle events
     - Включён через ``slam_launch.py`` только при ``use_slam=true``.
   * - ``/rviz2``
     - ``/map``, ``/scan``, ``/tf``, ``/tf_static`` и другие display topics
     - Запускается только при ``use_rviz=true``.

``spawn_rtk2026`` и два ``spawner`` выполняют одно действие и завершаются,
поэтому поздний ``rqt_graph`` их не показывает. ``/gz_ros_control`` может
выглядеть изолированным: основной обмен плагина проходит через hardware
interfaces, а не через обычные ROS-топики.

Практические команды для каждого launch и проверка получившегося графа:
:doc:`running`.

Исходники: `launch/ <https://github.com/Hanqnero/RTK2026/tree/main/src/rtk2026_bringup/launch>`_.
