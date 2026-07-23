Архитектура от команды до карты
===============================

Общая схема
-----------

На снимке показан исходный runtime-граф ``sim_slam_launch.py`` до включения
EKF. Эллипсы — ноды, прямоугольники — топики, стрелка идёт от publisher к
subscriber. Снимок сохранён как контрольная точка: в текущем графе между
``diff_drive_controller`` и TF появляется ``ekf_filter_node``.

.. figure:: /images/rqt_sim_slam_graph.png
   :alt: Граф нод и топиков симуляционного запуска RTK2026
   :width: 100%
   :class: rqt-graph

   Контрольный граф до EKF, без ``/tf``, ``/tf_static`` и неактивного
   publisher ``/cmd_vel``. Старый ``/odom`` в актуальном launch переименован
   в ``/wheel/odom`` и поступает в ``ekf_filter_node``.

.. important::

   ``rqt_graph`` показывает связи через ROS-топики, но не показывает
   hardware interfaces ``ros2_control``, вызовы сервисов, Pluginlib-связи и
   внутренний обмен Gazebo. Изолированная на снимке нода не обязательно
   бездействует.

Как возникают связи
-------------------

В ROS 2 нет общей команды «соединить ноду A с нодой B». Связь появляется,
когда одна нода публикует топик, а другая подписывается на то же имя с тем же
типом сообщения. Launch-файл определяет состав процессов и remap, YAML —
параметры готовых нод, URDF/Xacro — звенья и hardware interfaces.

Например, линия через ``/joint_states`` не означает, что
``/robot_description`` вычисляется из состояния колёс. Это два независимых
интерфейса ``robot_state_publisher``: нода читает суставы и отдельно публикует
развёрнутый URDF.

Блок модели и состояний суставов
--------------------------------

``joint_state_broadcaster -> /joint_states -> robot_state_publisher``
   ``gz_ros2_control`` читает position/velocity двух wheel joints из Gazebo.
   Загруженный ``JointStateBroadcaster`` публикует их как
   ``sensor_msgs/msg/JointState``. ``robot_state_publisher`` использует эти
   значения для динамических TF колёс.

``robot_state_publisher -> /robot_description -> controller_manager``
   Launch разворачивает ``rtk2026_diff_drive_sim.urdf.xacro`` в XML и передаёт
   строку параметром ``robot_description``. Нода публикует её с
   transient-local QoS. Созданный Gazebo-плагином ``controller_manager``
   получает оттуда блок ``<ros2_control>`` и описание joint interfaces.

``spawn_rtk2026``
   Процесс ``ros_gz_sim create`` также получает ``/robot_description``, создаёт
   модель и завершается. Поэтому на позднем снимке ``rqt_graph`` его нет.

Блок движения и одометрии
-------------------------

``teleop -> /cmd_vel -> diff_drive_controller``
   Внешний интерфейс симуляции — ``geometry_msgs/msg/TwistStamped``.
   Внутренний ``/diff_drive_controller/cmd_vel`` remap-ится в ``/cmd_vel``.
   Если teleop не запущен, у топика есть subscriber, но нет publisher, и
   ``rqt_graph`` обычно не рисует входную цепочку полностью.

``diff_drive_controller -> /wheel/odom -> ekf_filter_node``
   Контроллер берёт position feedback колёс через hardware interfaces,
   вычисляет сырую ``nav_msgs/msg/Odometry`` и публикует её после remap.
   ``ekf_filter_node`` использует скорости сообщения, публикует
   ``/odometry/filtered`` и единственный TF ``odom -> base_footprint``.
   У контроллера ``enable_odom_tf=false``.

``Gazebo IMU -> /imu/data -> ekf_filter_node``
   IMU system публикует angular velocity и acceleration относительно
   ``imu_link``. Bridge преобразует ``gz.msgs.IMU`` в
   ``sensor_msgs/msg/Imu``; EKF использует только ``angular_velocity.z``.

``slam_toolbox`` не обязан подписываться на ``/odometry/filtered``.
Предсказание позы он
получает из TF ``odom -> base_footprint`` на timestamp лазерного скана. Поэтому
на графе без TF-топиков нет стрелки от EKF к SLAM.

Блок лидара и карты
-------------------

``Gazebo lidar -> gazebo_bridge -> /scan -> slam_toolbox``
   GPU lidar сначала публикует Gazebo Transport ``gz.msgs.LaserScan``.
   ``ros_gz_bridge`` преобразует его в ``sensor_msgs/msg/LaserScan``.
   ``slam_toolbox`` подписывается на имя из ``scan_topic: /scan``.

``slam_toolbox -> /map``
   Активная lifecycle-нода публикует ``nav_msgs/msg/OccupancyGrid`` и TF
   ``map -> odom``. ``/slam_toolbox/transition_event`` и нода вида
   ``/launch_ros_<N>`` относятся к автоматическому управлению lifecycle, а
   ``/transform_listener_impl_<id>`` создаются библиотекой ``tf2_ros``.

Точные команды запуска, открытия ``rqt_graph`` и проверки каждой связи
собраны в :doc:`running`; полный контракт топиков — в :doc:`interfaces`.

Разделение ответственности
---------------------------

``arduino/``
   Временной цикл привода, чтение энкодеров, защита по таймауту, локальная
   остановка по сонару, PID, интегрирование колёсной одометрии и бинарный
   Serial-протокол.

``rtk2026_driver``
   Преобразование ROS-команды в ``ControlPacket`` и телеметрии Arduino в
   сырую ``nav_msgs/msg/Odometry``. В штатном bringup динамический TF
   публикует EKF, а не bridge.

``rtk2026_description``
   Геометрия, инерции и TF робота; отдельная модель реального робота и
   физическая модель дифференциального стенда; конфигурация
   ``ros2_control``.

``rtk2026_slam``
   Единственный владелец параметров ``slam_toolbox``.

``rtk2026_localization``
   Параметры локального EKF и AMCL. EKF публикует ``odom -> base_footprint``;
   в режиме известной карты AMCL вместо SLAM публикует ``map -> odom``.

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
