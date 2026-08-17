Локализация: EKF и метод частиц
================================

Пакет ``rtk2026_localization`` не содержит самописных математических
фильтров. Он задаёт проверяемые конфигурации и launch-обвязку для двух
готовых ROS 2-компонентов:

* ``robot_localization/ekf_node`` — непрерывная локальная одометрия;
* ``nav2_amcl`` — глобальная локализация частицами по известной карте.

Исходные файлы
--------------

* :download:`config/ekf.yaml <../src/rtk2026_localization/config/ekf.yaml>`;
* :download:`config/ekf_real.yaml <../src/rtk2026_localization/config/ekf_real.yaml>`;
* :download:`config/ekf_real_wheel_only.yaml <../src/rtk2026_localization/config/ekf_real_wheel_only.yaml>`;
* :download:`config/amcl.yaml <../src/rtk2026_localization/config/amcl.yaml>`;
* :download:`launch/ekf.launch.py <../src/rtk2026_localization/launch/ekf.launch.py>`;
* :download:`launch/particle_localization.launch.py <../src/rtk2026_localization/launch/particle_localization.launch.py>`.

Режимы работы
-------------

.. list-table:: Владелец каждой динамической трансформации
   :header-rows: 1
   :widths: 24 28 24 24

   * - Режим
     - ``odom -> base_footprint``
     - ``map -> odom``
     - Глобальный результат
   * - Построение карты
     - EKF
     - ``slam_toolbox``
     - новая ``/map``
   * - Известная карта
     - EKF
     - AMCL
     - поза на сохранённой ``/map``

.. warning::

   Не запускайте AMCL и ``slam_toolbox`` одновременно. Они будут
   конкурировать за один TF ``map -> odom``. Не включайте также
   ``diff_drive_controller.enable_odom_tf``: TF ``odom -> base_footprint``
   уже публикует EKF.

EKF локальной одометрии
-----------------------

Текущий поток данных симуляции:

.. code-block:: text

   motor joints ── diff_drive_controller ── /wheel/odom: vx, vy=0 ──┐
                                                                    ├──> ekf_filter_node
   Gazebo IMU ──────────────────────────── /imu/data: vyaw ──────────┘         ├── /odometry/filtered
                                                                              └── TF odom -> base_footprint

``/wheel/odom`` является рабочим входом симуляционного EKF. В
``robot_model:=diff_drive`` он получается из position state вращающихся
wheel joints — прямого аналога энкодерных углов. ``/ground_truth/odom`` в
фильтр не входит и используется только при оценке drift. В гусеничной
``tracked``-модели ``/wheel/odom`` пока вычисляет плагин ``TrackedVehicle``:
его контактные борта являются фиксированными links, а не motor joints.

``/odometry/filtered`` остаётся публичным выходом локального фильтра.
``world_frame=odom`` запрещает EKF создавать глобальные скачки: коррекции
карты остаются ответственностью SLAM или AMCL. EKF не калибрует радиус,
базу или знаки автоматически.

На реальном роботе используется отдельный ``ekf_real.yaml``:

.. code-block:: text

   Arduino bridge ── /wheel/odom: vx, vy=0 ──┐
                                             ├──> ekf_filter_node
   Raspberry Pi BMI270 ── /imu/data: vyaw ───┘         ├── /odometry/filtered
                                                       └── TF odom -> base_footprint

Физическая IMU присутствует в URDF как ``imu_link``, но статический link сам
по себе не создаёт измерений. Отдельная ROS-нода на Raspberry Pi должна
читать BMI270 по I²C и публиковать ``sensor_msgs/msg/Imu`` с
``header.frame_id=imu_link``. В штатном ``ekf_real.yaml`` включён gyro Z, а
wheel ``vyaw`` выключен.

``ekf_real_wheel_only.yaml`` оставлен как явно деградированный стендовый
режим без IMU. Он использует энкодерный ``vyaw`` и сильнее зависит от
проскальзывания и ошибки ``wheel_separation``.

Вектор ``odom0_config``
~~~~~~~~~~~~~~~~~~~~~~~

Порядок флагов фиксирован ``robot_localization``:

.. code-block:: text

   x, y, z,
   roll, pitch, yaw,
   vx, vy, vz,
   vroll, vpitch, vyaw,
   ax, ay, az

Текущая wheel-конфигурация включает ``vx`` и нулевое ``vy`` как
кинематическое ограничение дифференциального робота. Wheel ``vyaw`` выключен;
его заменяет IMU ``vyaw``. Pose из wheel odometry не объединяется с twist:
обе величины получены из одних joints и коррелируют.

.. list-table:: Важные параметры EKF
   :header-rows: 1
   :widths: 34 18 48

   * - Параметр
     - Значение
     - Смысл
   * - ``frequency``
     - 50 Гц
     - Частота predict/correct и публикации результата.
   * - ``sensor_timeout``
     - 0.2 с
     - После таймаута фильтр выполняет только predict.
   * - ``two_d_mode``
     - true
     - Фиксирует Z, roll и pitch для плоского движения.
   * - ``odom0``
     - ``/wheel/odom``
     - Продольная скорость по joint feedback или физическим энкодерам.
   * - ``imu0``
     - ``/imu/data``
     - Только ``angular_velocity.z`` относительно ``imu_link``.
   * - ``publish_tf``
     - true
     - EKF владеет ``odom -> base_footprint``.
   * - ``world_frame``
     - ``odom``
     - Результат остаётся непрерывным локальным estimate.
   * - ``use_control``
     - false
     - ``/cmd_vel`` не считается измерением движения.

AMCL: адаптивный фильтр частиц
--------------------------------------------------

AMCL использует известную ``nav_msgs/msg/OccupancyGrid``, LaserScan,
локальную одометрию через TF и начальную оценку позы. Результат —
``geometry_msgs/msg/PoseWithCovarianceStamped`` в ``/amcl_pose`` и
корректирующий TF ``map -> odom``.

.. code-block:: text

   map_server ── /map ──────────────┐
   lidar ─────── /scan ─────────────┼──> AMCL ── /amcl_pose
   EKF ───────── odom -> base TF ───┘          └── map -> odom TF

Количество частиц адаптивно меняется между ``min_particles=500`` и
``max_particles=2000`` методом KLD sampling. Модель
``DifferentialMotionModel`` соответствует дифференциальному приводу.

.. list-table:: Важные параметры AMCL
   :header-rows: 1
   :widths: 35 20 45

   * - Параметр
     - Значение
     - Что калибруется
   * - ``alpha1..alpha4``
     - 0.2
     - Шум вращения и перемещения в модели одометрии.
   * - ``update_min_d``
     - 0.05 м
     - Минимальное перемещение до laser update.
   * - ``update_min_a``
     - 0.05 рад
     - Минимальный поворот до laser update.
   * - ``laser_model_type``
     - ``likelihood_field``
     - Сравнение лучей с полем расстояний карты.
   * - ``max_beams``
     - 60
     - Число равномерно выбранных лучей на update.
   * - ``transform_tolerance``
     - 0.5 с
     - Допуск времени публикуемого ``map -> odom``.
   * - ``set_initial_pose``
     - false
     - Начальную позу обязан задать оператор.

Запуск построения карты
-----------------------

Текущий симуляционный launch уже включает EKF:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py

Проверьте единственных TF-publisher-ов и частоты:

.. code-block:: bash

   ros2 node info /ekf_filter_node
   ros2 topic hz /wheel/odom
   ros2 topic hz /imu/data
   ros2 topic hz /odometry/filtered
   ros2 topic echo /imu/data --once --field header
   ros2 topic echo /imu/data --once --field angular_velocity
   ros2 topic echo /diagnostics --once
   ros2 run tf2_ros tf2_echo odom base_footprint
   ros2 run tf2_ros tf2_monitor odom base_footprint

Для реального робота тот же контракт собирается одной командой:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_slam.py

Перед движением обязательны сообщения внешней Pi-ноды:

.. code-block:: bash

   ros2 topic hz /imu/data
   ros2 topic echo /imu/data --once --field header
   ros2 topic echo /imu/data --once --field angular_velocity_covariance

Запуск локализации по сохранённой карте
--------------------------------------------------

В первом терминале контейнера запустите физику, сенсоры и EKF без SLAM:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py slam_mode:=none

Во втором терминале активируйте Map Server и AMCL:

.. code-block:: bash

   ros2 launch rtk2026_localization particle_localization.launch.py \
     map:=/workspace/maps/my_map.yaml \
     use_sim_time:=true

Затем в RViz выберите ``2D Pose Estimate`` и укажите положение и направление
робота на карте. Эквивалентный интерфейс командной строки:

.. code-block:: bash

   ros2 topic pub --once \
     /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
     "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"

Диагностика particle localization:

.. code-block:: bash

   ros2 lifecycle get /map_server
   ros2 lifecycle get /amcl
   ros2 topic hz /amcl_pose
   ros2 topic echo /particle_cloud --once
   ros2 topic echo /amcl_pose --once
   ros2 run tf2_ros tf2_echo map odom
   ros2 topic info /tf -v

На реальном роботе аппаратные ноды, EKF, Map Server и AMCL объединены:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_localization.py \
     map:=/absolute/path/to/my_map.yaml \
     use_rviz:=true

Критерии качества
-----------------

* ``/wheel/odom`` и ``/odometry/filtered`` идут без пропусков и временных
  скачков;
* при остановке ``odom -> base_footprint`` не дрожит;
* облако частиц после начальной позы сходится к одной компактной области;
* лазер в RViz совпадает со стенами известной карты во время поворота;
* ``map -> odom`` может корректироваться, но ``odom -> base_footprint``
  остаётся непрерывным;
* в ``ros2 topic info /tf -v`` нет двух нод, публикующих один и тот же edge.

Для повторяемой настройки записывайте минимум:

.. code-block:: bash

   ros2 bag record \
     /clock /cmd_vel /wheel/odom /odometry/filtered \
     /scan /map /amcl_pose /particle_cloud /tf /tf_static

Ссылки
------

* `Nav2: настройка robot_localization <https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html>`_;
* `актуальный шаблон параметров robot_localization для ROS 2 <https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml>`_;
* `Nav2 AMCL parameters <https://docs.nav2.org/configuration/packages/configuring-amcl.html>`_;
* `Nav2 AMCL API для ROS 2 Jazzy <https://docs.ros.org/en/jazzy/p/nav2_amcl/>`_.
