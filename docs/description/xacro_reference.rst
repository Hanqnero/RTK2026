Справочник Xacro-макросов
=========================

Справочник макросов, из которых собраны модели. Нужен, когда правите URDF:
показывает, какие параметры принимает каждый макрос и что он создаёт.

Общие макросы инерции
---------------------

Файл
`common/inertials.xacro <https://github.com/Hanqnero/RTK2026/blob/main/src/rtk2026_description/urdf/common/inertials.xacro>`_.

``box_inertial(mass, length, width, height, cx, cy, cz)``
   Создаёт ``<inertial>`` однородного параллелепипеда. Размеры относятся к
   X/Y/Z соответственно; центр масс задаётся отдельно.

``sphere_inertial(mass, radius)``
   Однородный шар с ``Ixx=Iyy=Izz=2mr²/5``.

``cylinder_inertial(mass, radius, length, cx, cy, cz, rpy='0 0 0')``
   Цилиндр с локальной осью Z. ``rpy`` поворачивает inertial frame; это не
   visual-геометрия и не axis joint.

``wheel_inertial_y(mass, radius, width)``
   Специализированная инерция цилиндрического колеса, ось вращения которого Y.
   ``Iyy`` — осевой момент, ``Ixx/Izz`` — поперечные.

Материалы
---------

``common/materials.xacro`` объявляет ``black``, ``blue``, ``green``, ``gray``,
``darkgray``, ``red``, ``white`` и ``yellow``. Имена глобальны внутри итогового
URDF; макросы только ссылаются на них через ``<material name=...>``.

``rtk2026_tank_base_fixed``
----------------------------

Файл ``chassis/tank_base_fixed.xacro``. Создаёт fixed-модель реального шасси:

* ``base_footprint`` и ``base_link``;
* корпус с box collision и mesh/fallback visual;
* единый ``wheel_link`` всей ходовой части;
* fixed ``wheel_joint``.

.. list-table:: Параметры fixed chassis
   :header-rows: 1
   :widths: 38 62

   * - Параметры
     - Назначение
   * - ``prefix``
     - Префикс link/joint.
   * - ``base_mass``, ``base_length``, ``base_width``, ``base_height``
     - Масса и box-приближение корпуса.
   * - ``base_link_z``
     - Высота CAD frame над ``base_footprint``.
   * - ``base_com_x/y/z``
     - Центр масс корпуса.
   * - ``use_base_mesh``, ``base_mesh``
     - Выбор STL/fallback visual корпуса.
   * - ``use_running_gear_mesh``, ``running_gear_mesh``
     - Выбор STL/fallback visual и collision ходовой части.

``rtk2026_diffbot_base``
------------------------

Файл ``chassis/diffbot_base.xacro``. Создаёт физическую трёхточечную платформу
для Gazebo.

.. list-table:: Параметры diffbot
   :header-rows: 1
   :widths: 34 66

   * - Параметр
     - Использование
   * - ``prefix``
     - Префикс всех имён; default пустой.
   * - ``base_*``
     - Масса, размеры и высота корпуса.
   * - ``wheel_mass/radius/width``
     - Инерция и collision каждого ведущего колеса.
   * - ``wheel_separation``
     - Y-расстояние между колёсами.
   * - ``wheel_x``
     - X ведущей оси относительно центра ``base_link``; одновременно задаёт
       обратное смещение ``base_link`` относительно ``base_footprint``.
   * - ``wheel_effort/velocity``
     - URDF limits обоих wheel joints.
   * - ``caster_mass/radius/x``
     - Задняя фиксированная сферическая опора.
   * - ``use_base_mesh/base_mesh``
     - Visual корпуса; collision всегда box.

Surface friction задаётся через Gazebo extension: ведущие колёса ``1/1``,
caster ``0.001/0.001``. Collision колёс и caster не имеют явных ``name``: это
необходимо для корректной вставки ``surface/friction`` используемым
URDF→SDF-конвертером.

``rtk2026_robot``
-----------------

Главный макрос real-модели в ``robot_macro.xacro``. Параметры:
``prefix``, ``use_meshes``, ``use_webcam``. Он создаёт fixed chassis, IMU,
lidar и условно webcam. Внутренние properties являются предварительными
геометрическими значениями и требуют сверки с измерениями/CAD.

``rtk2026_diffbot_ros2_control``
--------------------------------

Параметры: ``prefix`` и ``max_velocity``. Создаёт system hardware
``gz_ros2_control/GazeboSimSystem`` и регистрирует для каждого wheel joint:

* command interface ``velocity`` с симметричными пределами;
* state interface ``position`` как энкодер;
* state interface ``velocity``.

Сам контроллер здесь не создаётся: его тип и параметры находятся в YAML, а
plugin верхнеуровневой модели создаёт ``controller_manager``.

Сенсорные макросы
-----------------

``rtk2026_imu(prefix, parent, xyz, rpy, use_mesh, mesh)``
   Создаёт ``imu_link`` и fixed ``imu_joint``. Visual выбирается между STL и
   box 40×30×12 мм; collision всегда box.

``rtk2026_imu_gazebo(prefix, update_rate, angular_velocity_stddev, linear_acceleration_stddev)``
   Сохраняет ``imu_link`` при URDF→SDF conversion и добавляет Gazebo sensor
   типа ``imu``. Публикует ``imu/data``; параметры noise задают σ независимого
   гауссовского шума каждой оси.

``rtk2026_lidar(prefix, parent, xyz, rpy, use_mesh, mesh, scan_yaw)``
   Создаёт физический ``lidar_link``, ``lidar_joint``, служебный
   ``lidar_frame`` и ``lidar_frame_joint``. ``scan_yaw`` корректирует нулевое
   направление LaserScan без поворота collision. ``preserveFixedJoint``
   сохраняет lidar link после конвертации в SDF.

``rtk2026_lidar_gazebo(...)``
   Добавляет ``gpu_lidar`` к существующему ``lidar_link``. Параметры:
   ``update_rate``, ``samples``, угловой диапазон, min/max range,
   ``range_resolution`` и ``noise_stddev``. Публикует Gazebo topic ``scan``.

``rtk2026_webcam(prefix, parent, xyz, rpy, mass, width, height, depth)``
   Создаёт box ``camera_link`` и REP-103 optical frame. Геометрическое
   соглашение camera link: X вперёд, Y влево, Z вверх; optical: Z вперёд,
   X вправо, Y вниз.

``rtk2026_webcam_gazebo(prefix, backend, width, height, update_rate, horizontal_fov)``
   Добавляет camera sensor и topic ``<prefix>webcam/image_raw``. При
   ``backend == 'classic'`` также добавляет ``libgazebo_ros_camera.so``.
   Это старый универсальный макрос; симуляционные модели RTK2026 вместо него
   используют D435i.

``sensor_d435i(parent, origin, name, use_nominal_extrinsics)``
   Официальный макрос пакета ``realsense2_description``. Создаёт mesh корпуса
   и nominal TF для color, depth, infra, accelerometer и gyroscope.

``rtk2026_realsense_d435i_gazebo(...)``
   Добавляет к официальному ``camera_link`` нативные сенсоры Gazebo Harmonic:
   синхронный ``rgbd_camera`` и встроенную IMU. Параметры задают resolution,
   частоты, horizontal FOV, clipping и σ шума IMU. Транспортные топики
   ``/camera/*`` приводит к ROS API RealSense единый ``sim_slam_launch.py``.
