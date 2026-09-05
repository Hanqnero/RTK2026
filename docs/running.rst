Запуск и наблюдение системы
===========================

Порядок запуска и остановки. Чем поднимается каждый слой по отдельности -
в :doc:`bringup`; как убедиться, что он работает, - в :doc:`diagnostics`.

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

На малинке с проброшенными ``/dev/arduino``, ``/dev/rplidar`` и
``/dev/i2c-1`` инерциальный датчик поднимается нодой ``imu_bridge``: она
читает BMI270 по I²C и публикует ``sensor_msgs/msg/Imu`` в ``/imu/data``
с ``frame_id=imu_link``. Отдельно её запускать не нужно - ``real_slam.py``
включает её при ``use_imu:=true``.

Проверьте IMU до общего launch:

.. code-block:: bash

   ros2 topic type /imu/data
   ros2 topic hz /imu/data
   ros2 topic echo /imu/data --once --field header
   ros2 topic echo /imu/data --once --field angular_velocity_covariance

После этого построение карты запускается так:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_slam.py

Если на том же X11-дисплее нужен RViz, создайте ``ros``
из графической сессии Pi:

.. code-block:: bash

   export DISPLAY
   export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
   docker compose -f pi/docker/docker-compose.pi.yml up -d --force-recreate ros

Затем запустите launch внутри уже созданного контейнера:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_slam.py use_rviz:=true

Compose передаёт сокет X11 и указанный cookie-файл в
``ros``. Не запускайте compose через ``sudo``: так теряются
``DISPLAY`` и ``XAUTHORITY``. Для обычного Xorg fallback — ``:0`` и
``$HOME/.Xauthority``; Wayland/Xwayland обычно сам задаёт другой
``XAUTHORITY`` в ``/run/user/<uid>/``.

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

Где что искать
--------------

Разделы, которые раньше жили здесь, переехали к своим владельцам: копии
расходятся, а ссылки нет.

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - Что нужно
     - Где смотреть
   * - Аргументы и состав launch-файлов
     - :doc:`bringup`
   * - Список топиков, типы, системы координат
     - :doc:`interfaces`
   * - Проверка узлов, топиков, частот, граф системы
     - :doc:`diagnostics`
   * - Дерево TF и кто публикует каждое ребро
     - :doc:`architecture`
   * - Контроллеры и hardware interfaces
     - :doc:`description/controllers`
   * - Запись эксперимента в bag
     - :doc:`diagnostics`
