Архитектура от команды до карты
===============================

Общая схема
-----------

Два варианта исполнения сохраняют одинаковые ROS-интерфейсы выше уровня
привода:

.. code-block:: text

   teleop / навигация
          │
          ▼
   /cmd_vel : TwistStamped
          │
          ├── реальный робот ──> ArduinoBridgeNode ──USB Serial──> Arduino
          │                                              │
          │                                              └── энкодеры, PID,
          │                                                  одометрия
          │
          └── симуляция ──────> diff_drive_controller ──> Gazebo joints
                                                        │
                                                        └── joint feedback

   ArduinoBridgeNode или diff_drive_controller
          ├── /odom : Odometry
          └── odom -> base_footprint

   лидар
          └── /scan : LaserScan

   slam_toolbox
          ├── map -> odom
          └── /map : OccupancyGrid

Разделение ответственности
---------------------------

``arduino/``
   Временной цикл привода, чтение энкодеров, защита по таймауту, локальная
   остановка по сонару, PID, интегрирование колёсной одометрии и бинарный
   Serial-протокол.

``rtk2026_driver``
   Преобразование ROS-команды в ``ControlPacket`` и телеметрии Arduino в
   ``nav_msgs/msg/Odometry`` и динамический TF.

``rtk2026_description``
   Геометрия, инерции и TF робота; отдельная модель реального робота и
   физическая модель дифференциального стенда; конфигурация
   ``ros2_control``.

``rtk2026_slam``
   Единственный владелец параметров ``slam_toolbox``.

``rtk2026_bringup``
   Композиция готовых компонентов. Bringup не дублирует конфигурации драйвера
   и SLAM.

``docker/``
   Воспроизводимое окружение ROS 2 Jazzy + Gazebo Harmonic + RViz и доступ к
   экрану RViz через noVNC.

Дерево TF
---------

При SLAM дерево должно иметь один путь между каждой парой фреймов:

.. code-block:: text

   map               
   └── odom                   
       └── base_footprint      
           └── base_link       
               ├── imu_link
               ├── lidar_link
               │   └── lidar_frame
               └── camera_link
                   └── camera_optical_frame

В симуляционной модели |base_footprint| расположен на плоскости пола под
серединой ведущей оси.

Источники времени
-----------------

* Реальный робот использует системные часы ROS: ``use_sim_time=false``.
* Симуляция получает время из ``/clock``: ``use_sim_time=true``.
* Нельзя смешивать источники времени у лидара, TF и ``slam_toolbox`` — иначе
  сканы будут отбрасываться из-за отсутствующего преобразования на их метке
  времени.
