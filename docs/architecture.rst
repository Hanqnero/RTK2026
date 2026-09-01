Архитектура от команды до карты
===============================

Как команда движения доходит до колёс, а лазерные сканы превращаются в карту.
Страница отвечает на вопрос, кто с кем связан и кто публикует каждое ребро
дерева координат. Имена и типы сообщений - в :doc:`interfaces`, команды
проверки - в :doc:`diagnostics`.

Общая схема
-----------

На снимке показан исходный runtime-граф колёсного варианта
``sim_slam_launch.py`` до включения EKF. Эллипсы — ноды, прямоугольники —
топики, стрелка идёт от publisher к subscriber. Снимок сохранён как
контрольная точка; текущий default ``robot_model:=tracked`` имеет вместо
``diff_drive_controller`` ноду ``tracked_drive_bridge``.

.. figure:: /images/rqt_sim_slam_graph.png
   :alt: Граф нод и топиков симуляционного запуска RTK2026
   :width: 100%
   :class: rqt-graph

   Контрольный граф до EKF, без ``/tf``, ``/tf_static`` и неактивного
   publisher ``/cmd_vel``. Старый ``/odom`` в актуальном launch переименован
   в ``/wheel/odom``. EKF читает этот единый интерфейс симуляционного и
   реального привода; ``/ground_truth/odom`` используется только диагностикой.

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

При ``robot_model:=tracked`` Gazebo получает физическую SDF-модель, а
``robot_state_publisher`` — отдельный Xacro с тем же TF-деревом. Условные
``controller_manager`` и wheel joints ниже существуют только при
``robot_model:=diff_drive``.

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

``diff_drive_controller -> /wheel/odom``
   Контроллер берёт position feedback колёс через hardware interfaces,
   вычисляет сырую ``nav_msgs/msg/Odometry`` и публикует её после remap.
   EKF использует из него ``vx`` и кинематическое ограничение ``vy=0``.
   У контроллера ``enable_odom_tf=false``.

``teleop -> /cmd_vel -> tracked_drive_bridge -> TrackedVehicle``
   Default-гусеничная модель передаёт ``TwistStamped`` в Gazebo Transport.
   ``TrackedVehicle`` рассчитывает скорости двух ``TrackController``.

``wheel joints -> /wheel/odom -> ekf_filter_node``
   В варианте ``diff_drive`` штатный ``diff_drive_controller`` рассчитывает
   одометрию по position feedback двух motor joints. В ``tracked`` тот же
   интерфейс предоставляет ``TrackedVehicle``: у его контактных бортов пока
   нет вращающихся joints. Фильтр публикует ``/odometry/filtered`` и
   единственный TF ``odom -> base_footprint``.

``Gazebo ground truth -> /ground_truth/odom -> diagnostics``
   Идеальная физическая поза не участвует в EKF или SLAM. Она нужна только
   для оценки drift и калибровки геометрии привода.

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

В режиме ``slam_mode:=visual`` lidar-ветка глобального картирования
заменяется готовым RTAB-Map:

``RGB-D + /odometry/filtered -> /rtabmap/rtabmap -> /map``
   RTAB-Map синхронизирует color, aligned depth и camera info, использует
   непрерывную одометрию EKF и публикует единственный ``map -> odom``.
   Собственная visual odometry отключена, поэтому второго
   ``odom -> base_footprint`` не возникает.

Точные команды запуска, открытия ``rqt_graph`` и проверки каждой связи
собраны в :doc:`running`; полный контракт топиков — в :doc:`interfaces`.

Разделение ответственности
---------------------------

``arduino/``
   Временной цикл привода, чтение энкодеров, защита по таймауту, локальная
   остановка по сонару, PID, интегрирование колёсной одометрии и бинарный
   Serial-протокол.

``rtk2026_driver``
   Преобразование ROS-команды в кадр ``CMD_VELOCITY`` и телеметрии Arduino в
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
                   ├── camera_color_frame
                   │   └── camera_color_optical_frame
                   ├── camera_depth_frame
                   │   └── camera_depth_optical_frame
                   ├── camera_infra1/infra2_frame
                   └── camera_accel/gyro_frame

В симуляционной модели |base_footprint| расположен на плоскости пола под
серединой ведущей оси.

Источники времени
-----------------

* Реальный робот использует системные часы ROS: ``use_sim_time=false``.
* Симуляция получает время из ``/clock``: ``use_sim_time=true``.
* Нельзя смешивать источники времени у лидара, TF и ``slam_toolbox`` — иначе
  сканы будут отбрасываться из-за отсутствующего преобразования на их метке
  времени.
