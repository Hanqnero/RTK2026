Верхнеуровневые модели
======================

Реальный робот
--------------

Файл
`rtk2026_real.urdf.xacro <https://github.com/Hanqnero/RTK2026/blob/main/src/rtk2026_description/urdf/rtk2026_real.urdf.xacro>`_
является тонкой оболочкой над ``rtk2026_robot``.

.. list-table:: Аргументы real-модели
   :header-rows: 1

   * - Аргумент
     - Default
     - Назначение
   * - ``prefix``
     - пусто
     - Префикс всех link/joint для нескольких экземпляров.
   * - ``use_meshes``
     - true
     - STL visual вместо упрощённых примитивов.
   * - ``use_webcam``
     - true
     - Добавить camera frames и геометрию.

Макрос ``rtk2026_robot`` задаёт предварительные параметры корпуса:

.. list-table:: Real model properties
   :header-rows: 1

   * - Имя
     - Значение
     - Комментарий
   * - ``base_mass``
     - 1.6 кг
     - Масса корпуса без отдельных датчиков.
   * - ``base_length`` / ``width`` / ``height``
     - 0.30 / 0.15 / 0.10 м
     - Размер fallback collision/visual.
   * - ``base_link_z``
     - 0.127 м
     - Высота CAD-origin над полом.
   * - ``base_com``
     - (0, 0, -0.13) м
     - Предварительный центр масс; требует проверки.

Дерево состоит только из fixed joints:

.. code-block:: text

   base_footprint
   └── base_link
       ├── wheel_link
       ├── imu_link
       ├── lidar_link
       │   └── lidar_frame
       └── camera_link
           └── camera_optical_frame

Поскольку joint фиксированы, ``joint_state_publisher`` не требуется.
``robot_state_publisher`` получает описание через ``display.launch.py``.

Дифференциальная симуляция
--------------------------

Файл
`rtk2026_diff_drive_sim.urdf.xacro <https://github.com/Hanqnero/RTK2026/blob/main/src/rtk2026_description/urdf/rtk2026_diff_drive_sim.urdf.xacro>`_
собирает ``diffbot_base``, сенсоры и ``diffbot_ros2_control``.

.. list-table:: Геометрия и динамика симуляции
   :header-rows: 1
   :widths: 34 20 46

   * - Property
     - Значение
     - Значение в модели
   * - ``base_mass``
     - 1.6 кг
     - Масса box корпуса.
   * - ``base_length`` / ``width`` / ``height``
     - 0.300 / 0.240 / 0.113 м
     - Корпус.
   * - ``base_link_z``
     - 0.08 м
     - Высота origin корпуса над полом.
   * - ``wheel_radius``
     - 0.06 м
     - Геометрия и controller config должны совпадать.
   * - ``wheel_width``
     - 0.030 м
     - Ширина collision cylinder.
   * - ``wheel_separation``
     - 0.246 м
     - Расстояние между центрами левого и правого колеса.
   * - ``wheel_x``
     - 0.06 м
     - Ведущая ось впереди центра корпуса.
   * - ``wheel_mass``
     - 0.155 кг
     - Масса одного ведущего колеса.
   * - ``wheel_max_effort``
     - 5.0
     - URDF effort limit.
   * - ``wheel_max_velocity``
     - 30 рад/с
     - URDF и ros2_control velocity limit.
   * - ``caster_radius`` / ``mass``
     - 0.06 м / 0.02 кг
     - Задняя сферическая опора.
   * - ``caster_x``
     - 0.09 м
     - Опора позади центра корпуса.

Положение относительно ``base_footprint``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|base_footprint| находится под ведущей осью. Поэтому:

.. code-block:: text

   base_link        x=-0.060, y=0,      z=0.080
   left_wheel       x= 0,     y=+0.123, z=0.060
   right_wheel      x= 0,     y=-0.123, z=0.060
   rear_caster      x=-0.150, y=0,      z=0.060

Оси обоих wheel joint — ``0 1 0``. Положительная позиция обоих энкодеров
соответствует движению вперёд по +X. Visual/collision cylinder поворачивается
на +π/2 вокруг X, чтобы геометрическая ось Z цилиндра совпала с Y робота.

Gazebo plugins
~~~~~~~~~~~~~~

``GazeboSimROS2ControlPlugin`` загружает
``config/diffbot_controllers.yaml`` и создаёт ``controller_manager``.
Официальный ``sensor_d435i`` из ``realsense2_description`` создаёт геометрию
и TF камеры, а ``rtk2026_realsense_d435i_gazebo`` добавляет RGB-D и IMU
потоки Gazebo Harmonic.
``OdometryPublisher`` публикует идеальную позу в Gazebo Transport
``/ground_truth/odom`` с частотой 50 Гц и без шума. ``sim_slam_launch.py``
автоматически мостит его в одноимённый ROS-топик только для диагностики; TF
из ground truth не публикуется.

Отдельная модель камеры
-----------------------

``rtk2026_webcam.urdf.xacro`` создаёт минимальный родительский link и вызывает
``rtk2026_webcam``. Она предназначена для проверки геометрии/TF камеры без
полного робота. Аргументы: ``parent``, ``xyz`` и ``rpy``.
