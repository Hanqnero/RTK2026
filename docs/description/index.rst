Описание робота и симуляция
===========================

``rtk2026_description`` содержит две намеренно разные модели:

``rtk2026_real.urdf.xacro``
   Фиксированная геометрическая модель реального гусеничного робота для TF и
   RViz. Вращаемых колёс и ``ros2_control`` нет.

``rtk2026_diff_drive_sim.urdf.xacro``
   Физический дифференциальный стенд Gazebo с двумя вращаемыми колёсами,
   пассивной опорой, lidar sensor и ``gz_ros2_control``.

.. toctree::
   :maxdepth: 2

   models
   xacro_reference
   controllers
   sensors
   world

Правило выбора модели
---------------------

Используйте real-модель для отображения фактического робота, где одометрия
приходит из Arduino. Используйте diff-drive-модель только в Gazebo. Нельзя
добавлять симуляционные wheel joints и ``gz_ros2_control`` в real-модель:
тогда одно описание начнёт одновременно изображать CAD-шасси и другую
кинематическую схему.

Проверка Xacro
--------------

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

   xacro \
     "$(ros2 pkg prefix --share rtk2026_description)/urdf/rtk2026_diff_drive_sim.urdf.xacro" \
     use_meshes:=false > /tmp/rtk2026.urdf

   check_urdf /tmp/rtk2026.urdf
   gz sdf -p /tmp/rtk2026.urdf > /tmp/rtk2026.sdf

``check_urdf`` проверяет URDF и дерево связей. ``gz sdf -p`` дополнительно
показывает результат преобразования, реально используемый Gazebo: оси joint,
fixed-joint lumping, surface friction, sensors и plugins.
