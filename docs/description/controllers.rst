ros2_control и дифференциальный привод
======================================

Конфигурация:
`diffbot_controllers.yaml <https://github.com/Hanqnero/RTK2026/blob/main/src/rtk2026_description/config/diffbot_controllers.yaml>`_.

Controller manager
------------------

``update_rate: 100`` задаёт частоту чтения Gazebo state interfaces, обновления
контроллеров и записи wheel velocity commands. ``joint_state_broadcaster``
публикует ``position`` и ``velocity`` обоих joints в ``/joint_states``.

DiffDriveController
-------------------

.. list-table:: Основные параметры
   :header-rows: 1
   :widths: 38 22 40

   * - Параметр
     - Значение
     - Назначение
   * - ``left_wheel_names``
     - ``left_wheel_joint``
     - Левый feedback/command joint.
   * - ``right_wheel_names``
     - ``right_wheel_joint``
     - Правый feedback/command joint.
   * - ``wheel_separation``
     - 0.246 м
     - Номинальная колея.
   * - ``wheel_radius``
     - 0.060 м
     - Номинальный радиус.
   * - ``wheel_separation_multiplier``
     - 1.0
     - Калибровка угла поворота.
   * - ``left/right_wheel_radius_multiplier``
     - 1.0 / 1.0
     - Калибровка масштаба и асимметрии колёс.
   * - ``position_feedback``
     - true
     - Одометрия использует накопленную позицию joint.
   * - ``open_loop``
     - false
     - Команды не подменяют обратную связь.
   * - ``publish_rate``
     - 50 Гц
     - Частота ``/odom`` и TF.
   * - ``cmd_vel_timeout``
     - 0.5 с
     - Нулевая команда после потери input.
   * - ``velocity_rolling_window_size``
     - 10
     - Усреднение публикуемой скорости.

Кинематика
----------

Эффективные размеры:

.. math::

   r_l = r\,m_l, \quad r_r = r\,m_r, \quad b_{eff}=b\,m_b

Одометрический поворот:

.. math::

   \Delta\theta = \frac{r_r\Delta\phi_r-r_l\Delta\phi_l}{b_{eff}}

При завышенном угле нужно увеличить ``wheel_separation_multiplier``; при
заниженном — уменьшить. Полная процедура приведена в :doc:`../calibration`.

Frames и TF
-----------

``odom_frame_id=odom``, ``base_frame_id=base_footprint`` и
``enable_odom_tf=true`` означают, что контроллер является единственным
издателем ``odom -> base_footprint``. Нельзя одновременно публиковать этот TF
другим узлом одометрии.

Ограничения команды
-------------------

.. code-block:: text

   linear.x  ∈ [-1.0, +1.0] m/s
   angular.z ∈ [-3.0, +3.0] rad/s
   linear acceleration ≤ 1.5 m/s²
   angular acceleration ≤ 4.0 rad/s²

``publish_limited_velocity=true`` позволяет видеть применённую после limits
команду в ``/diff_drive_controller/cmd_vel_out``.

Ковариации
-----------

Обе диагонали заданы как ``[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]`` в
порядке X, Y, Z, roll, pitch, yaw. Это заявленная постоянная неопределённость,
а не измеренная статистика и не матрица scan correlation.
