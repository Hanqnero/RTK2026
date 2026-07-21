ROS-интерфейсы и соглашения
===========================

Топики
------

.. list-table:: Основные топики
   :header-rows: 1
   :widths: 22 32 18 28

   * - Имя
     - Тип
     - Источник
     - Назначение
   * - ``/cmd_vel``
     - ``geometry_msgs/msg/TwistStamped``
     - teleop или навигация
     - ``twist.linear.x`` и ``twist.angular.z``; остальные компоненты приводом
       не используются.
   * - ``/odom``
     - ``nav_msgs/msg/Odometry``
     - Arduino bridge или ``diff_drive_controller``
     - Локальная непрерывная колёсная одометрия.
   * - ``/scan``
     - ``sensor_msgs/msg/LaserScan``
     - RPLIDAR или мост Gazebo
     - Вход ``slam_toolbox``; ожидаемый ``frame_id`` — ``lidar_frame``.
   * - ``/joint_states``
     - ``sensor_msgs/msg/JointState``
     - ``joint_state_broadcaster``
     - Положения и скорости симулируемых колёс.
   * - ``/encoder_joint_states``
     - ``sensor_msgs/msg/JointState``
     - ``quantize_joint_states``
     - Диагностическая копия с разрешением реального энкодера.
   * - ``/ground_truth/odom``
     - ``nav_msgs/msg/Odometry`` после ``ros_gz_bridge``
     - Gazebo ``OdometryPublisher``
     - Только калибровка симуляции; не является входом SLAM.
   * - ``/map``
     - ``nav_msgs/msg/OccupancyGrid``
     - ``slam_toolbox``
     - Текущая растровая карта.
   * - ``/clock``
     - ``rosgraph_msgs/msg/Clock``
     - Gazebo
     - Симуляционное время.

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
