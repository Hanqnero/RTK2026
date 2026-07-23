ROS-интерфейсы и соглашения
===========================

Топики
------

.. list-table:: Основные топики
   :header-rows: 1
   :widths: 18 27 19 19 27

   * - Имя
     - Тип
     - Publisher
     - Основной subscriber
     - Назначение
   * - ``/cmd_vel``
     - ``geometry_msgs/msg/TwistStamped``
     - teleop или навигация
     - ``diff_drive_controller`` или ``arduino_bridge``
     - ``twist.linear.x`` и ``twist.angular.z``; остальные компоненты приводом
       не используются.
   * - ``/diff_drive_controller/cmd_vel_out``
     - ``geometry_msgs/msg/TwistStamped``
     - ``diff_drive_controller``
     - диагностика/bag
     - Команда после velocity/acceleration limits и timeout.
   * - ``/wheel/odom``
     - ``nav_msgs/msg/Odometry``
     - ``diff_drive_controller`` или ``arduino_bridge``
     - EKF/диагностика
     - Сырая одометрия колёс: joint feedback в Gazebo или телеметрия
       энкодеров на реальном роботе.
   * - ``/odometry/filtered``
     - ``nav_msgs/msg/Odometry``
     - ``ekf_filter_node``
     - RViz/диагностика
     - Непрерывный локальный estimate; соответствует TF
       ``odom -> base_footprint``.
   * - ``/imu/data``
     - ``sensor_msgs/msg/Imu``
     - Gazebo bridge; позже драйвер BMI270
     - EKF/диагностика
     - EKF использует ``angular_velocity.z`` в frame ``imu_link``.
   * - ``/scan``
     - ``sensor_msgs/msg/LaserScan``
     - RPLIDAR или мост Gazebo
     - ``slam_toolbox``
     - Вход ``slam_toolbox``; ожидаемый ``frame_id`` — ``lidar_frame``.
   * - ``/joint_states``
     - ``sensor_msgs/msg/JointState``
     - ``joint_state_broadcaster``
     - ``robot_state_publisher``
     - Положения и скорости симулируемых колёс.
   * - ``/robot_description``
     - ``std_msgs/msg/String``
     - ``robot_state_publisher``
     - ``controller_manager``; при старте ``spawn_rtk2026``
     - Развёрнутый URDF. Не является результатом вычисления
       ``/joint_states``.
   * - ``/encoder_joint_states``
     - ``sensor_msgs/msg/JointState``
     - ``quantize_joint_states``
     - диагностика
     - Диагностическая копия с разрешением реального энкодера.
   * - ``/ground_truth/odom``
     - ``nav_msgs/msg/Odometry`` после отдельного bridge
     - Gazebo ``OdometryPublisher`` при его включении
     - диагностика
     - Truth только для симуляционной калибровки. Текущий sim launch мостит
       его автоматически и не создаёт из него TF.
   * - ``/map``
     - ``nav_msgs/msg/OccupancyGrid``
     - ``slam_toolbox`` или ``map_server``
     - RViz, AMCL, map tools
     - Строящаяся либо заранее сохранённая растровая карта.
   * - ``/amcl_pose``
     - ``geometry_msgs/msg/PoseWithCovarianceStamped``
     - AMCL
     - RViz/навигация/диагностика
     - Глобальная оценка позы частицами по известной карте.
   * - ``/particle_cloud``
     - ``nav2_msgs/msg/ParticleCloud``
     - AMCL
     - RViz/диагностика
     - Текущее распределение частиц.
   * - ``/clock``
     - ``rosgraph_msgs/msg/Clock``
     - ``gazebo_bridge``
     - все ноды с ``use_sim_time=true``
     - Симуляционное время.
   * - ``/tf``
     - ``tf2_msgs/msg/TFMessage``
     - SLAM или AMCL, EKF, ``robot_state_publisher``
     - transform listeners
     - Динамические ``map -> odom``, ``odom -> base_footprint`` и wheel TF.
   * - ``/tf_static``
     - ``tf2_msgs/msg/TFMessage``
     - ``robot_state_publisher``
     - transform listeners
     - Фиксированные преобразования корпуса и сенсоров.

Блоки runtime-графа
-------------------

Модель робота
   ``joint_state_broadcaster -> /joint_states -> robot_state_publisher`` и
   ``robot_state_publisher -> /robot_description -> controller_manager``.

Привод
   ``teleop -> /cmd_vel -> diff_drive_controller -> /wheel/odom -> EKF``;
   параллельно ``Gazebo IMU -> /imu/data -> EKF``.
   Связь локализации со SLAM/AMCL идёт через скрытый на обычном ``rqt_graph``
   TF ``odom -> base_footprint``.

Реальный привод
   ``teleop/navigation -> /cmd_vel -> arduino_bridge -> Serial`` и
   ``Arduino telemetry -> /wheel/odom -> EKF``. Независимая нода на Raspberry
   Pi читает BMI270 по I²C и публикует ``/imu/data -> EKF``; Arduino bridge в
   тракт IMU не входит.

Сенсор и SLAM
   ``gazebo_bridge -> /scan -> slam_toolbox -> /map``. Дополнительно SLAM
   публикует TF ``map -> odom`` и lifecycle transition events.

Известная карта
   ``map_server -> /map -> AMCL`` и ``gazebo_bridge -> /scan -> AMCL``.
   AMCL публикует ``/amcl_pose``, ``/particle_cloud`` и TF ``map -> odom``.

Фактический publisher/subscriber всегда проверяется командой
``ros2 topic info <имя> -v``. Полный набор команд приведён в :doc:`running`.

Системы координат
-----------------

``map``
   Глобальная система карты. Может скачкообразно корректироваться после
   scan matching и замыкания цикла.

``odom``
   Непрерывная локальная система колесной одометрии. Не должна совершать
   скачков, но может постепенно накапливать дрейф.

``base_footprint``
   Плоская проекция центра движения. Для дифференциальной модели совпадает по
   X/Y с серединой ведущей оси.

``base_link``
   Система корпуса, относительно которой заданы датчики и CAD-геометрия.

``lidar_frame``
   Система, записываемая в ``LaserScan.header.frame_id``. ``scan_yaw`` в Xacro
   позволяет согласовать нулевой луч драйвера с направлением +X робота.

``camera_optical_frame``
   Оптическое соглашение ROS: Z вперёд, X вправо, Y вниз.

Единицы и знаки
---------------

* расстояния и линейные скорости: метр, метр в секунду;
* углы и угловые скорости: радиан, радиан в секунду;
* положительный X: вперёд;
* положительный Y: влево;
* положительный Z: вверх;
* положительный ``angular.z``: поворот против часовой стрелки при взгляде
  сверху;
* положительные показания обоих энкодеров должны соответствовать движению
  робота вперёд.

Ссылки на внешние спецификации
------------------------------

* `diff_drive_controller для ROS 2 Jazzy <https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html>`_;
* `slam_toolbox <https://docs.ros.org/en/jazzy/p/slam_toolbox/>`_;
* `интеграция ROS 2 и Gazebo <https://gazebosim.org/docs/harmonic/ros2_integration/>`_;
* `расширения URDF тегами Gazebo <https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/1.6/>`_.
