Запуск и наблюдение системы
===========================

В этом разделе команды разделены по месту выполнения:

``HOST``
   Терминал macOS/Linux в корне репозитория.

``CONTAINER``
   Интерактивный Bash внутри ``rtk2026_sim``. В нём уже подключены
   ``/opt/ros/jazzy`` и ``/workspace/install`` через ``/root/.bashrc``.

Запуск контейнера
-----------------

Новый entrypoint запускает только Xvfb, Fluxbox, x11vnc и noVNC. ROS, Gazebo,
SLAM и RViz автоматически не стартуют; контейнер удерживается командой
``sleep infinity``.

HOST — собрать и поднять контейнер в фоне:

.. code-block:: bash

   docker compose -f docker/docker-compose.sim.yml up -d --build
   docker compose -f docker/docker-compose.sim.yml ps

Открыть виртуальный экран:

.. code-block:: text

   http://127.0.0.1:6080/vnc.html  # Gazebo и RViz
   http://127.0.0.1:6081/vnc.html  # RQt и PlotJuggler

HOST — войти в контейнер:

.. code-block:: bash

   docker exec -it rtk2026_sim bash

При сомнении проверить окружение в CONTAINER:

.. code-block:: bash

   printenv ROS_DISTRO
   ros2 pkg prefix rtk2026_bringup
   ros2 pkg executables rtk2026_driver

Основные launch-файлы
---------------------

.. list-table:: Точки входа проекта
   :header-rows: 1
   :widths: 34 30 36

   * - Команда в CONTAINER
     - Что запускает
     - Когда использовать
   * - ``ros2 launch rtk2026_bringup sim_slam_launch.py``
     - Gazebo, модель, bridge, ros2_control, EKF, SLAM и RViz
     - Полная симуляционная проверка картирования.
   * - ``ros2 launch rtk2026_bringup sim_slam_launch.py use_rviz:=false``
     - Тот же стек без RViz
     - Headless-тест частот и топиков.
   * - ``ros2 launch rtk2026_bringup sim_slam_launch.py slam_mode:=none``
     - Gazebo, привод, сенсоры, EKF и RViz без SLAM
     - Базовый стек перед AMCL по известной карте.
   * - ``ros2 launch rtk2026_bringup sim_slam_launch.py slam_mode:=visual``
     - RealSense D435i, RTAB-Map RGB-D, EKF и RViz
     - Визуальное картирование с сохранением графа в базе RTAB-Map.
   * - ``ros2 launch rtk2026_bringup sim_slam_launch.py robot_model:=diff_drive``
     - Тот же стек с колёсной моделью и ``diff_drive_controller``
     - Сравнение колёсной и гусеничной ходовой части.
   * - ``ros2 launch rtk2026_localization particle_localization.launch.py map:=/workspace/maps/my_map.yaml use_sim_time:=true``
     - Map Server, AMCL и lifecycle manager
     - Запускать вторым процессом после базового стека без SLAM.
   * - ``ros2 launch rtk2026_bringup real_slam.py``
     - Real URDF, Arduino bridge, RPLIDAR, EKF и SLAM
     - Полный запуск на ПК/роботе с подключёнными устройствами.
   * - ``ros2 launch rtk2026_bringup real_localization.py map:=/absolute/path/my_map.yaml``
     - Те же реальные датчики и EKF, затем Map Server + AMCL
     - Движение по уже построенной карте; после старта задать initial pose.
   * - ``ros2 launch rtk2026_bringup arduino_launch.py``
     - Только ``arduino_bridge``
     - Проверка Serial, команд и энкодерной одометрии.
   * - ``ros2 launch rtk2026_bringup lidar_launch.py``
     - Только официальный драйвер RPLIDAR C1
     - Проверка ``/scan`` реального лидара.
   * - ``ros2 launch rtk2026_bringup slam_launch.py use_sim_time:=false``
     - Только ``slam_toolbox``
     - Когда ``/scan`` и полное TF-дерево уже существуют.
   * - ``ros2 launch rtk2026_description display.launch.py``
     - Real Xacro и ``robot_state_publisher``
     - Проверка геометрии и статических TF без привода.
   * - ``ros2 launch rtk2026_observability diagnostics_lidar.launch.py use_gui:=true``
     - Диагностика SLAM Toolbox, RQt и PlotJuggler
     - Запускать рядом с ``slam_mode:=lidar``.
   * - ``ros2 launch rtk2026_observability diagnostics_visual.launch.py use_gui:=true``
     - Диагностика RGB-D и RTAB-Map, RQt и PlotJuggler
     - Запускать рядом с ``slam_mode:=visual``.
   * - ``ros2 launch rtk2026_observability diagnostics.launch.py use_gui:=true localization_mode:=localization``
     - Диагностика Map Server + AMCL
     - Запускать рядом с sim-локализацией без SLAM.

``lidar_launch.py`` требует установленного ``sllidar_ros2`` и устройство
``/dev/rplidar``. Симуляционный Dockerfile намеренно пропускает эту внешнюю
зависимость при ``rosdep install``, поэтому реальные launch-файлы проверяются
и запускаются в ROS-окружении робота, а не в текущем sim image.

Полная симуляция
----------------

CONTAINER — стандартный мир:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py

По умолчанию запускается гусеничная модель, а EKF использует её
``/wheel/odom`` и основную ``/imu/data``. Для проверки именно вращающихся
motor joints и ``diff_drive_controller`` передайте
``robot_model:=diff_drive``. Launch занимает текущий терминал и пишет туда
логи. Для teleop, ``rqt_graph``
и диагностики откройте второй HOST-терминал и снова войдите в тот же контейнер:

.. code-block:: bash

   docker exec -it rtk2026_sim bash

Произвольный world передаётся абсолютным путём:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/absolute/path/to/world.sdf

``polygon_5x5.world`` копируется в образ вместе с каталогом ``worlds``:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/workspace/worlds/polygon_5x5.world

Электролизный мир и гусеничная модель выбраны по умолчанию. Рабочая площадка
поднята до ``z=0.08``, а исходная ``spawn_z=0.081`` оставляет зазор 1 мм.
Чтобы направить модель на блок из 12 балок, достаточно изменить Y:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     spawn_y:=0.5725

Визуальный SLAM в том же мире выбирается только параметром
``slam_mode:=visual``. Окно Gazebo по умолчанию не создаётся; цветное
изображение и occupancy grid отображаются в общем RViz:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     slam_mode:=visual

Значение ``spawn_yaw=pi/2`` по умолчанию направляет переднюю D435i вдоль
центрального прохода к задней стене. По её центру находится один стандартный
OpenCV ArUco ``DICT_4X4_50`` с ``id=0``. У модели маркера нет collision,
поэтому она не изменяет коллизии полигона.

Проверить RGB-D входы и ноду RTAB-Map:

.. code-block:: bash

   ros2 topic hz /camera/color/image_raw
   ros2 topic hz /camera/aligned_depth_to_color/image_raw
   ros2 topic hz /camera/color/camera_info
   ros2 topic hz /rtabmap/info
   ros2 node info /rtabmap/rtabmap
   ros2 topic echo /map --once
   ros2 run tf2_ros tf2_echo map odom

RTAB-Map получает непрерывную локальную одометрию из
``/odometry/filtered`` и синхронизирует color, aligned depth и
``camera_info``. Основная ``/imu/data`` уже участвует в EKF и второй раз
не подаётся в граф. База по умолчанию сохраняется в
``/workspace/records/rtabmap/rtk2026.db``.

Окна независимо включаются аргументами ``use_gazebo_gui`` и ``use_rviz``.

Локализация частицами в этом мире выполняется двумя командами в разных
CONTAINER-терминалах:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/workspace/worlds/polygon_5x5.world \
     slam_mode:=none

   ros2 launch rtk2026_localization particle_localization.launch.py \
     map:=/workspace/maps/my_map.yaml \
     use_sim_time:=true

Управление TwistStamped
-----------------------

``diff_drive_controller`` в ROS 2 Jazzy принимает только
``geometry_msgs/msg/TwistStamped``. Поэтому teleop нужно явно переключить в
stamped-режим:

.. code-block:: bash

   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
     --ros-args \
     -p stamped:=true \
     -p use_sim_time:=true \
     -p frame_id:=base_footprint \
     --remap cmd_vel:=/cmd_vel

Параметры ``stamped`` и ``frame_id`` описаны в
`документации teleop_twist_keyboard для Jazzy <https://docs.ros.org/en/ros2_packages/jazzy/api/teleop_twist_keyboard/__README.html>`_.
Последняя скорость сохраняется до следующей команды; ``space`` или ``k``
публикуют остановку.

Проверить тип до начала движения:

.. code-block:: bash

   ros2 topic type /cmd_vel
   ros2 topic info /cmd_vel -v

Короткую постоянную команду можно подать без teleop. Команда выполняется до
``Ctrl+C``; timeout контроллера остановит робот после прекращения публикации:

.. code-block:: bash

   ros2 topic pub --rate 10 \
     /cmd_vel geometry_msgs/msg/TwistStamped \
     "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.10}, angular: {z: 0.0}}}"

Для поворота на месте замените twist на
``linear.x: 0.0, angular.z: 0.30``.

Открытие rqt_graph
------------------

После запуска симуляции выполните во втором CONTAINER-терминале:

.. code-block:: bash

   rqt_graph

Окно появится в noVNC. Для графа, похожего на приведённый в архитектуре:

* включите отображение нод и топиков;
* скройте debug-топики;
* временно скройте ``/tf`` и ``/tf_static``, чтобы увидеть основной data flow;
* запустите teleop, если нужна видимая цепочка через ``/cmd_vel``.

``rqt_graph`` не показывает hardware interfaces и краткоживущие процессы
``spawn_rtk2026``/``spawner`` после их завершения. TF отдельно удобнее смотреть
через ``view_frames`` или ``rqt_tf_tree``.

Диагностическое GUI
-------------------

Для SLAM Toolbox откройте третий CONTAINER-терминал:

.. code-block:: bash

   ros2 launch rtk2026_observability diagnostics_lidar.launch.py \
     use_gui:=true

Для визуального SLAM:

.. code-block:: bash

   ros2 launch rtk2026_observability diagnostics_visual.launch.py \
     use_gui:=true

Для AMCL измените профиль:

.. code-block:: bash

   ros2 launch rtk2026_observability diagnostics.launch.py \
     use_gui:=true \
     localization_mode:=localization

``rqt_robot_monitor`` показывает агрегированное дерево
``/diagnostics_agg``, ``rqt_runtime_monitor`` — исходные статусы
``/diagnostics``, PlotJuggler — временные графики выбранных полей топиков.
Окна находятся на ``http://127.0.0.1:6081/vnc.html``.

Текущие автоматические профили observability рассчитаны на Gazebo: они
проверяют ``/imu/data``, ``/wheel/odom`` и ground truth. На реальном роботе до
появления отдельного real-профиля используйте проверки частот, нод и TF из
этого раздела; иначе симуляционные ожидания закономерно дадут stale/warn.

Ноды и их интерфейсы
--------------------

Список активных нод:

.. code-block:: bash

   ros2 node list | sort

Подробно проверить главные блоки:

.. code-block:: bash

   ros2 node info /joint_state_broadcaster
   ros2 node info /robot_state_publisher
   ros2 node info /controller_manager
   ros2 node info /diff_drive_controller
   ros2 node info /ekf_filter_node
   ros2 node info /gazebo_bridge
   ros2 node info /slam_toolbox

Состояние lifecycle SLAM:

.. code-block:: bash

   ros2 lifecycle get /slam_toolbox
   ros2 topic info /slam_toolbox/transition_event -v

Топики и реальные связи
-----------------------

Полный список с типами:

.. code-block:: bash

   ros2 topic list -t

Publisher, subscriber, тип и QoS для каждого основного топика:

.. code-block:: bash

   for topic in \
       /joint_states \
       /robot_description \
       /cmd_vel \
       /wheel/odom \
       /imu/data \
       /odometry/filtered \
       /scan \
       /map \
       /clock \
       /tf \
       /tf_static
   do
       echo "========== ${topic} =========="
       ros2 topic info "${topic}" -v || true
   done

Разовые сообщения без бесконечного вывода:

.. code-block:: bash

   ros2 topic echo /joint_states --once
   ros2 topic echo /robot_description --once
   ros2 topic echo /wheel/odom --once
   ros2 topic echo /imu/data --once
   ros2 topic echo /odometry/filtered --once
   ros2 topic echo /scan --once --field header
   ros2 topic echo /map --once --field info

Частоты и пропускная способность
--------------------------------

Измерять нужно после стабилизации Gazebo, отдельно в спокойном состоянии и во
время движения:

.. code-block:: bash

   ros2 topic hz /clock
   ros2 topic hz /joint_states
   ros2 topic hz /wheel/odom
   ros2 topic hz /imu/data
   ros2 topic hz /odometry/filtered
   ros2 topic hz /scan
   ros2 topic hz /map
   ros2 topic bw /scan
   ros2 topic bw /map

.. list-table:: Ожидаемые порядки частот симуляции
   :header-rows: 1
   :widths: 25 22 53

   * - Интерфейс
     - Ожидание
     - Откуда берётся
   * - ros2_control update
     - 100 Гц
     - ``controller_manager.update_rate``; это цикл control, не топик.
   * - ``/wheel/odom``
     - 50 Гц
     - ``diff_drive_controller.publish_rate``.
   * - ``/odometry/filtered``
     - 50 Гц
     - ``ekf_filter_node.frequency``.
   * - ``/imu/data``
     - 100 Гц
     - ``rtk2026_imu_gazebo.update_rate``.
   * - ``/scan``
     - 10 Гц
     - ``update_rate`` Gazebo lidar.
   * - обработка сканов SLAM
     - не чаще 5 Гц
     - ``minimum_time_interval: 0.2``; отдельного выходного топика нет.
   * - ``/map``
     - до 2 Гц
     - ``map_update_interval: 0.5``.
   * - ``/joint_states``
     - измерить фактически
     - broadcaster работает внутри update loop, отдельная частота не задана.

``ros2 topic hz /tf`` смешивает сообщения нескольких broadcaster-ов и не
равен частоте одной трансформации.

Контроллеры и hardware interfaces
---------------------------------

.. code-block:: bash

   ros2 control list_controllers
   ros2 control list_hardware_components
   ros2 control list_hardware_interfaces -v
   ros2 param dump /diff_drive_controller
   ros2 param dump /ekf_filter_node
   ros2 topic echo /diff_drive_controller/cmd_vel_out --once

Ожидаются активные ``joint_state_broadcaster`` и ``diff_drive_controller``.
Velocity command interfaces колёс должны иметь отметку ``[claimed]``.

TF и координаты
---------------

.. code-block:: bash

   ros2 run tf2_ros tf2_echo map odom
   ros2 run tf2_ros tf2_echo odom base_footprint
   ros2 run tf2_ros tf2_echo base_footprint base_link
   ros2 run tf2_ros tf2_echo base_link lidar_frame
   ros2 run tf2_tools view_frames
   ros2 run rqt_tf_tree rqt_tf_tree

``view_frames`` создаёт ``frames.pdf`` в текущем каталоге. Для SLAM критично,
чтобы на timestamp каждого ``/scan`` существовал единственный путь
``map -> odom -> base_footprint -> ... -> lidar_frame``.

Запись воспроизводимого эксперимента
------------------------------------

.. code-block:: bash

   ros2 bag record \
     /clock /cmd_vel /diff_drive_controller/cmd_vel_out \
     /joint_states /wheel/odom /imu/data /odometry/filtered \
     /scan /map /tf /tf_static

Сохранение карты и pose graph
-----------------------------

Каталог ``/workspace/maps`` смонтирован в ``maps/`` репозитория на HOST,
поэтому файлы сразу остаются на компьютере после остановки контейнера.
Во время активного mapping сохраните растровую карту:

.. code-block:: bash

   ros2 run nav2_map_server map_saver_cli \
     -f /workspace/maps/my_map \
     --ros-args -p use_sim_time:=true

Команда создаёт ``my_map.yaml`` и ``my_map.pgm``. Для продолжения
картирования средствами ``slam_toolbox`` отдельно сохраните сериализованный
pose graph:

.. code-block:: bash

   ros2 service call \
     /slam_toolbox/serialize_map \
     slam_toolbox/srv/SerializePoseGraph \
     "{filename: '/workspace/maps/my_map'}"

Появятся ``my_map.posegraph`` и ``my_map.data``. AMCL использует только
``my_map.yaml`` + ``my_map.pgm``; pose graph нужен не AMCL, а
``slam_toolbox`` для продолжения/редактирования сессии.

Реальный робот
--------------

На ROS-компьютере с установленным ``sllidar_ros2`` и проброшенными
``/dev/arduino`` и ``/dev/rplidar`` сначала запустите отдельную ноду BMI270,
которая читает I²C Raspberry Pi и публикует
``sensor_msgs/msg/Imu`` в ``/imu/data`` с ``frame_id=imu_link``. Эта
hardware-нода пока не реализована в выбранных пакетах проекта.

Проверьте IMU до общего launch:

.. code-block:: bash

   ros2 topic type /imu/data
   ros2 topic hz /imu/data
   ros2 topic echo /imu/data --once --field header
   ros2 topic echo /imu/data --once --field angular_velocity_covariance

После этого построение карты запускается так:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_slam.py

Если на том же X11-дисплее нужен RViz:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_slam.py use_rviz:=true

Для локализации по сохранённой карте SLAM запускать нельзя:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_localization.py \
     map:=/absolute/path/to/my_map.yaml \
     use_rviz:=true

После активации AMCL задайте начальную позу кнопкой ``2D Pose Estimate``.
До движения проверьте реальные частоты и единственность TF:

.. code-block:: bash

   ros2 topic hz /wheel/odom
   ros2 topic hz /odometry/filtered
   ros2 topic hz /scan
   ros2 topic info /tf -v
   ros2 run tf2_ros tf2_echo odom base_footprint

Только для стендовой проверки без BMI270 можно явно выбрать деградированный
конфиг, использующий wheel ``vyaw``:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_slam.py \
     ekf_config:="$(ros2 pkg prefix --share rtk2026_localization)/config/ekf_real_wheel_only.yaml"

Этот режим хуже переносит проскальзывание и ошибку фактической колеи; он не
является штатным вариантом картирования.

Остановка
---------

Сначала нажмите ``Ctrl+C`` в терминалах teleop и launch, затем выйдите из
контейнера. HOST:

.. code-block:: bash

   docker compose -f docker/docker-compose.sim.yml down
