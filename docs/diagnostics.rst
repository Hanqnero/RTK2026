Диагностика системы
===================

Порядок проверки плохой карты
-----------------------------

Проверяйте снизу вверх:

1. wheel joints и знаки энкодеров;
2. фактическое движение против ``/odom``;
3. статические TF датчиков;
4. timestamp и frame_id ``/scan``;
5. только после этого — параметры scan matching.

Топики и частоты
----------------

.. code-block:: bash

   ros2 topic list
   ros2 topic info -v /cmd_vel
   ros2 topic hz /odom
   ros2 topic hz /scan
   ros2 topic echo /joint_states --once
   ros2 topic echo /scan --once --field header

Для симуляции nominal: controller update 100 Гц, odom 50 Гц, lidar 10 Гц,
SLAM input не чаще 5 Гц из-за ``minimum_time_interval=0.2``.

TF
--

.. code-block:: bash

   ros2 run tf2_ros tf2_echo odom base_footprint
   ros2 run tf2_ros tf2_echo base_footprint base_link
   ros2 run tf2_ros tf2_echo base_link lidar_frame
   ros2 run tf2_tools view_frames

Матрица ``tf2_echo`` имеет вид ``[R t; 0 1]`` и описывает одно жёсткое
преобразование. Повторяющиеся значения после остановки показывают отсутствие
дрожания, но не доказывают точность. Для качества нужны дельты во время
манёвра и независимый truth.

Проверка URDF→SDF
-----------------

.. code-block:: bash

   xacro \
     "$(ros2 pkg prefix --share rtk2026_description)/urdf/rtk2026_diff_drive_sim.urdf.xacro" \
     > /tmp/robot.urdf
   gz sdf -p /tmp/robot.urdf > /tmp/robot.sdf

   grep -n -A8 -B3 '<surface>' /tmp/robot.sdf
   grep -n -A10 "joint name='left_wheel_joint'" /tmp/robot.sdf
   grep -n -A10 "joint name='right_wheel_joint'" /tmp/robot.sdf

Ожидается:

* обе axis: ``0 1 0``;
* wheel friction: ``mu=mu2=1``;
* caster friction: ``mu=mu2=0.001``;
* центры колёс X=0 относительно ``base_footprint``.

Контроллеры
-----------

.. code-block:: bash

   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ros2 param get /diff_drive_controller wheel_separation
   ros2 param get /diff_drive_controller wheel_radius
   ros2 topic echo /diff_drive_controller/cmd_vel_out

Оба контроллера должны быть ``active``. ``cmd_vel_out`` показывает команду
после acceleration/velocity limits и timeout.

RViz для одометрии
------------------

Установите Fixed Frame ``odom`` и добавьте два Odometry display:
``/odom`` и ``/ground_truth/odom``. На чистом повороте стрелки должны менять
yaw вокруг одной точки. Сантиметровое смещение ``base_footprint`` при повороте
указывает на неправильный центр frame или проскальзывание.

Запись эксперимента
-------------------

.. code-block:: bash

   ros2 bag record \
     /clock /cmd_vel /diff_drive_controller/cmd_vel_out \
     /joint_states /odom /ground_truth/odom /scan /tf /tf_static

В bag должны попасть команда, применённая команда, feedback, обе одометрии,
лидар и TF. Без этого невозможно разделить проблему управления, физики и SLAM.

Docker
------

.. code-block:: bash

   docker logs --tail 200 rtk2026_sim
   docker exec rtk2026_sim bash -lc \
     'source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; ros2 node list'
   docker exec rtk2026_sim bash -lc 'tail -100 /tmp/xvfb.log'
   docker exec rtk2026_sim bash -lc 'tail -100 /tmp/novnc.log'

Ошибки Mesa/RViz относятся к отображению; неверный ``map -> odom`` при
повороте обычно относится к odometry/scan matching. Эти классы проблем нужно
анализировать раздельно.
