Диагностика системы
===================

Команды первичного запуска, ``rqt_graph``, обхода нод и измерения частот
собраны в :doc:`running`. Здесь приведён порядок локализации неисправности и
более узкие проверки модели/одометрии.

Порядок проверки плохой карты
-----------------------------

Проверяйте снизу вверх:

1. направление отклика ``/wheel/odom`` на ``/cmd_vel``;
2. фактическое движение против ``/wheel/odom`` и ``/odometry/filtered``;
3. статические TF датчиков;
4. timestamp и frame_id ``/scan``;
5. только после этого — параметры scan matching.

Топики и частоты
----------------

.. code-block:: bash

   ros2 topic list
   ros2 topic info -v /cmd_vel
   ros2 topic hz /wheel/odom
   ros2 topic hz /imu/data
   ros2 topic hz /odometry/filtered
   ros2 topic hz /scan
   ros2 topic echo /imu/data --once
   ros2 topic echo /scan --once --field header

Для основной tracked-симуляции nominal: odom 50 Гц, lidar 10 Гц. Только у
дополнительной ``diff_drive``-модели controller update равен 100 Гц.
SLAM Toolbox принимает измерения не чаще 5 Гц из-за
``minimum_time_interval=0.2``.

Автоматическое дерево диагностики
---------------------------------

Пакет ``rtk2026_observability`` разделяет live-проверки на независимые
ветки:

``Compute``
   CPU, RAM и свободное место в каталоге записей.

``Topics``
   Наличие, частота и свежесть ``/scan``, ``/imu/data``, одометрии,
   ``/map`` и других интерфейсов. ``/pose`` от ``slam_toolbox`` является
   событийным: неподвижный робот не обязан публиковать его с постоянной
   частотой.

``Nodes``
   Наличие обязательных нод, сервисы, lifecycle-состояние и недавние
   ``WARN``/``ERROR`` из ``/rosout``.

``TF``
   Связность ``map -> odom -> base_footprint`` и ветвей лидара/IMU,
   свежесть динамических преобразований, смена parent и скачки позы.

``Time``
   Нулевые и обратные timestamp, ``frame_id`` и рассинхронизация
   ``scan``, IMU, wheel odometry и выхода EKF.

``Sensors``
   Геометрия и значения ``LaserScan``, ковариации и шум IMU на стоянке,
   конечность ``/wheel/odom`` и соответствие направления движения команде
   ``/cmd_vel``.

``Localization``
   Матрицы ковариации wheel odometry/EKF/SLAM Toolbox/AMCL, согласованность
   скоростей EKF с колёсами и gyro Z, пригодность occupancy grid и
   статистика particle cloud AMCL. В симуляции также показывается текущая
   ошибка wheel odometry и EKF относительно ``/ground_truth/odom``. Перед
   сравнением каждая траектория приводится к собственной первой позе, поэтому
   spawn-сдвиг и spawn-yaw не считаются ошибкой одометрии. Сюда же
   агрегируются штатные статусы ``robot_localization``.

Lidar-профиль требует активный ``slam_toolbox``, а visual-профиль —
``/rtabmap/rtabmap``, ``/rtabmap/info``, RGB-D topics и ``/map``. Тяжёлые
RGB/depth изображения не передаются через Python-монитор: проверяются их
publishers и subscribers в ROS graph. В
``localization`` она автоматически переключается на lifecycle-ноды
``map_server`` и ``amcl``; одновременно требовать SLAM и AMCL не будет.
Проверка topics также переключается: обновляемая карта SLAM контролируется
по возрасту, а статическая карта Map Server считается корректной после
одного transient-local сообщения. Выходы ``/amcl_pose`` и
``/particle_cloud`` считаются событийными.

.. note::

   Эти готовые YAML-профили относятся к tracked-симуляции и используют IMU,
   расчётную ``/wheel/odom`` и ``/ground_truth/odom``. Их нельзя без
   изменений считать
   диагностикой реального робота: до отдельного real-профиля отсутствующие
   симуляционные источники будут отмечаться как stale.

SLAM Toolbox и lidar-топики:

.. code-block:: bash

   ros2 launch rtk2026_observability diagnostics_lidar.launch.py \
     use_gui:=true

RTAB-Map и RGB-D-топики:

.. code-block:: bash

   ros2 launch rtk2026_observability diagnostics_visual.launch.py \
     use_gui:=true

В отдельном AMCL-запуске используется общий launch:

.. code-block:: bash

   ros2 launch rtk2026_observability diagnostics.launch.py \
     use_gui:=true localization_mode:=localization

Для ``slam_toolbox`` ветка ``Nodes/slam_toolbox`` проверяет:

* наличие ``/slam_toolbox`` в ROS graph;
* lifecycle-состояние ``active``;
* сервисы ``get_state``, ``pause_new_measurements`` и ``serialize_map``;
* число ``WARN`` и ``ERROR/FATAL`` с начала запуска;
* текст и возраст последнего ещё актуального предупреждения.

Для RTAB-Map ветка ``Nodes/rtabmap`` проверяет присутствие ноды и её
``WARN``/``ERROR`` из ``/rosout``. ``Topics/rtabmap_info`` подтверждает
обработку RGB-D кадров, а ``Topics/map`` — публикацию результата. Это
диагностика ROS-интерфейсов, а не внутренняя метрика качества feature
matching или loop closure.

Проверка без RQt:

.. code-block:: bash

   ros2 topic echo /diagnostics_agg --once --field status
   ros2 topic echo /diagnostics --once --field status
   ros2 lifecycle get /slam_toolbox
   ros2 service list | grep /slam_toolbox

``/diagnostics`` содержит исходные статусы, ``/diagnostics_agg`` — дерево
``RTK2026`` для ``rqt_robot_monitor``. Значения ``Correlation X-Y``,
``Correlation X-yaw`` и ``Correlation Y-yaw`` — нормированные внедиагональные
элементы covariance в диапазоне от -1 до 1. Знак показывает направление
совместной ошибки, а модуль около 1 — сильную связанность оценок. Это не
корреляция карты с лидаром и не метрика scan matching.

У ``/wheel/odom`` и локального EKF нет абсолютного измерения ``x``, ``y`` и
``yaw``. Поэтому их ``sigma`` закономерно растут во времени: они выводятся в
дерево и на графики, но сами по себе не создают ERROR. Числовые пределы
неопределённости применяются только к глобальным оценкам SLAM и AMCL.

``slam_toolbox`` не публикует через стандартный API числовые response каждого
scan matching или готовую метрику качества loop closure. Такие величины нельзя
надёжно восстановить из одного ``/diagnostics``. Для них нужен записанный
эксперимент: ``/scan``, ``/pose``, ``/map``, TF, одометрия, ground truth и
``/rosout``.

Снимок графа для отчёта
-----------------------

Во втором терминале контейнера запустите ``rqt_graph`` и откройте окно через
noVNC. Для основного data flow скройте ``/tf`` и ``/tf_static``; для поиска
пропавшего преобразования, наоборот, включите их или используйте
``rqt_tf_tree``. Отсутствие ``spawn_rtk2026`` и controller spawner-ов после
старта нормально: это завершившиеся служебные процессы.

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

Установите Fixed Frame ``odom`` и добавьте три Odometry display:
``/wheel/odom``, ``/odometry/filtered`` и ``/ground_truth/odom``. На чистом
повороте стрелки должны менять yaw вокруг одной точки. Сантиметровое смещение
``base_footprint`` при повороте указывает на неправильный центр frame или
проскальзывание.

Запись эксперимента
-------------------

.. code-block:: bash

   ros2 bag record \
     /clock /cmd_vel /wheel/odom /odometry/filtered \
     /ground_truth/odom /scan /tf /tf_static

В bag tracked-модели должны попасть команда, обе одометрии, lidar и TF.
Для отдельного эксперимента ``robot_model:=diff_drive`` дополнительно
запишите ``/joint_states`` и ``/diff_drive_controller/cmd_vel_out``.

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
