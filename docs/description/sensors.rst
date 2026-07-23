Датчики и их системы координат
==============================

IMU
---

В real-модели IMU расположен относительно ``base_link``:

.. code-block:: text

   xyz = 0.0048416 0.011168 -0.0057398
   rpy = π 0 -π/2

В diff-drive-симуляции упрощённый IMU находится по центру у нижней грани
корпуса. ``imu.xacro`` создаёт инерцию, visual, collision и TF, а
``imu_gazebo.xacro`` добавляет sensor Gazebo Harmonic.

.. list-table:: Симуляционная IMU
   :header-rows: 1

   * - Параметр
     - Значение
   * - Gazebo system
     - ``gz::sim::systems::Imu``
   * - Частота
     - 100 Гц
   * - Frame
     - ``imu_link``
   * - Gazebo topic
     - ``/imu/data``
   * - ROS topic после bridge
     - ``/imu/data`` типа ``sensor_msgs/msg/Imu``
   * - σ angular velocity
     - 0.002 рад/с на каждую ось
   * - σ linear acceleration
     - 0.02 м/с² на каждую ось

EKF использует только ``angular_velocity.z``. Orientation и acceleration
публикуются для диагностики, но пока не входят в fusion. Шумы являются
стартовой моделью симуляции, а не результатом калибровки реального BMI270.

Лидар
-----

Real-модель сохраняет предварительную CAD-позу; diff-drive-модель ставит lidar
в центре верхней грани корпуса. ``lidar_frame`` отделён от физического
``lidar_link``, чтобы направление скана можно было исправлять ``scan_yaw``.

Параметры симуляционного lidar:

.. list-table:: GPU lidar
   :header-rows: 1

   * - Параметр
     - Значение
   * - Частота
     - 10 Гц
   * - Samples
     - 720
   * - Угол
     - от -π до +π
   * - Дальность
     - 0.12…12.0 м
   * - Разрешение дальности
     - 0.01 м
   * - Gaussian σ
     - 0.005 м
   * - Topic Gazebo
     - ``scan``
   * - ROS topic после bridge
     - ``/scan``

.. note::

   Текущий ``<gz_frame_id>`` копируется конвертером как неизвестное SDF child
   и может выдавать warning. Фактический ``LaserScan.header.frame_id`` следует
   проверять командой ``ros2 topic echo /scan --once --field header``.

Камера
------

В diff-drive-модели камера установлена по центру передней грани:

.. math::

   x = base\_length/2 + camera\_depth/2 = 0.1625\;m

Она создаёт ``camera_link`` и ``camera_optical_frame``. Sensor macro
``webcam_gazebo.xacro`` существует, но не подключён в верхнеуровневую модель;
поэтому image topic пока отсутствует.

Связь с SLAM
------------

Для 2D SLAM критичны три условия:

1. ``LaserScan.header.frame_id`` существует в TF;
2. преобразование ``base_footprint -> lidar_frame`` доступно на timestamp
   каждого скана;
3. плоскость сканирования параллельна полу, а +X фрейма соответствует
   направлению, принятому в модели.
