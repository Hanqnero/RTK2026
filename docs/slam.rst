Параметры SLAM
==============

Пакет ``rtk2026_slam`` не содержит собственного алгоритма SLAM. Он является
единственным владельцем конфигурации ``slam_toolbox``; запуск выполняет
``rtk2026_bringup/slam_launch.py`` через официальный
``online_async_launch.py``.

Runtime-блок ноды:

.. code-block:: text

   /scan ──> slam_toolbox ──> /map
                │
                ├── TF map -> odom
                └── /slam_toolbox/transition_event

Одометрическое предсказание приходит через TF ``odom -> base_footprint``, а не
через прямую подписку ``slam_toolbox`` на ``/wheel/odom`` или
``/odometry/filtered``. Поэтому обычный
``rqt_graph`` со скрытыми TF показывает только цепочку ``/scan -> /map``.

Интерфейсы
----------

.. list-table:: Входы и выходы
   :header-rows: 1

   * - Интерфейс
     - Значение
     - Роль
   * - ``scan_topic``
     - ``/scan``
     - Лазерные измерения.
   * - ``odom_frame``
     - ``odom``
     - Непрерывная исходная оценка движения.
   * - ``base_frame``
     - ``base_footprint``
     - Плоская поза робота.
   * - ``map_frame``
     - ``map``
     - Глобальная система создаваемой карты.
   * - TF output
     - ``map -> odom``
     - Коррекция дрейфа одометрии scan matching и pose graph.
   * - Map output
     - ``/map``
     - ``nav_msgs/msg/OccupancyGrid`` с разрешением 0.05 м.

Режим и частоты
---------------

``mode: mapping`` создаёт новую карту и расширяет pose graph.
``throttle_scans: 1`` допускает каждый входной скан, но
``minimum_time_interval: 0.2`` ограничивает добавление измерений примерно
5 Гц. Симуляционный лидар публикует 10 Гц; поэтому часть сканов намеренно не
входит в обработку.

``transform_publish_period: 0.02`` публикует ``map -> odom`` до 50 Гц.
``map_update_interval: 0.5`` обновляет OccupancyGrid до 2 Гц. Эти частоты не
следует путать с частотой scan matching.

Локальный scan matching
-----------------------

.. list-table:: Основные параметры локального сопоставления
   :header-rows: 1
   :widths: 38 16 46

   * - Параметр
     - Значение
     - Смысл
   * - ``use_scan_matching``
     - true
     - Уточнять odometry prediction по текущему скану.
   * - ``use_scan_barycenter``
     - true
     - Использовать геометрический центр скана как опорную точку.
   * - ``minimum_travel_distance``
     - 0.05 м
     - Минимальное перемещение до нового узла графа.
   * - ``minimum_travel_heading``
     - 0.05 рад
     - Минимальный поворот, около 2.86°.
   * - ``scan_buffer_size``
     - 10
     - Размер локальной цепочки последних сканов.
   * - ``scan_buffer_maximum_scan_distance``
     - 10 м
     - Максимальная дистанция до скана локального буфера.
   * - ``link_scan_maximum_distance``
     - 1.5 м
     - Максимальная дистанция обычного последовательного ограничения.
   * - ``link_match_minimum_response_fine``
     - 0.1
     - Минимальный отклик точного сопоставления соседних сканов.

Пространство корреляционного поиска
-----------------------------------

``correlation_search_space_dimension = 0.5`` задаёт линейный размер локальной
области поиска, ``resolution = 0.01`` — шаг, ``smear_deviation = 0.1`` —
размытие отклика. Это **не ковариационная матрица ROS**: здесь correlation —
оценка совпадения лазерного скана с локальной картой при переборе поз.

Угловой поиск использует:

* ``fine_search_angle_offset = 0.00349`` рад;
* ``coarse_search_angle_offset = 0.349`` рад;
* ``coarse_angle_resolution = 0.0349`` рад.

Если ошибка одометрии между двумя сканами выходит за пространство поиска,
scan matcher может выбрать неверный локальный максимум. Поэтому систематическая
ошибка ``wheel_separation`` исправляется в приводе, а не расширением поиска.

Замыкание цикла
---------------

``do_loop_closing: true`` включает loop closure. Кандидаты ищутся до 3 м,
минимальная цепочка содержит 10 сканов. Грубое совпадение должно иметь response
не ниже 0.35, точное — 0.45. Область loop search имеет размер 8 м и шаг 0.05 м.

Слишком слабые пороги дают ложные замыкания и деформируют карту; слишком
жёсткие не исправляют накопленный дрейф. Менять их следует только после
проверки TF, лидара и колёсной одометрии.

Штрафы и оптимизатор
--------------------

``distance_variance_penalty`` и ``angle_variance_penalty`` снижают оценку
решений, удалённых от odometry prediction. Pose graph оптимизируется Ceres:

.. code-block:: text

   solver_plugin        = solver_plugins::CeresSolver
   linear_solver        = SPARSE_NORMAL_CHOLESKY
   preconditioner       = SCHUR_JACOBI
   trust_strategy       = LEVENBERG_MARQUARDT
   loss_function        = None

Параметры LaserScan и TF
------------------------

Рабочий диапазон ``0.12…12.0`` м должен совпадать с драйвером/симуляционным
сенсором. ``transform_timeout=0.2`` ограничивает ожидание TF, а
``tf_buffer_duration=30`` хранит историю. ``scan_queue_size=1`` сохраняет
актуальный скан вместо накопления устаревшей очереди.

Запуск
------

Реальный робот:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_slam.py

Симуляция:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py

В симуляции ``use_sim_time`` принудительно true, на реальном роботе должен
быть false. Исходный YAML:
`slam_toolbox_params.yaml <https://github.com/Hanqnero/RTK2026/blob/main/src/rtk2026_slam/config/slam_toolbox_params.yaml>`_.
Команды проверки lifecycle, входного скана, карты и TF: :doc:`running`.
