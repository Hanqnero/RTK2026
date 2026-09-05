Контейнер симуляции и noVNC
===========================

Образ предназначен для Apple Silicon и обычного Linux: базовый
``ros:jazzy-ros-base-noble`` имеет ARM64-вариант, а GUI работает через
виртуальный X11 и программный Mesa OpenGL.

Слои Dockerfile
---------------

1. Устанавливается ROS 2 desktop, Gazebo integration, ``gz_ros2_control``,
   контроллеры, ``slam_toolbox``, ``robot_localization``, компоненты AMCL и
   ``teleop_twist_keyboard``.
2. Устанавливается графический стек ``Xvfb → fluxbox → x11vnc → websockify``.
3. Compose монтирует ``src/``, ``maps/`` и ``worlds/`` в
   ``/workspaces/sim_ws``.
4. ``rosdep`` разрешает зависимости, кроме внешнего ``sllidar_ros2``.
5. Entrypoint поднимает виртуальный экран.
6. Default command собирает ``sim_ws``, подключает overlay и
   запускает Bash.

Исходный каталог хоста смонтирован в контейнер. ``--symlink-install`` позволяет
править Python-код без пересборки образа; при изменении CMake-пакетов достаточно
перезапустить контейнер или вручную повторить ``colcon build``.

Состав runtime
--------------

.. list-table:: Процессы entrypoint
   :header-rows: 1
   :widths: 24 24 52

   * - Процесс
     - Интерфейс
     - Назначение
   * - ``Xvfb``
     - DISPLAY ``:1``
     - Экран 1600×900×24, GLX и XRender.
   * - ``fluxbox``
     - X11
     - Оконный менеджер для RViz.
   * - ``x11vnc``
     - localhost:5900
     - Экспортирует X11, допускает несколько клиентов, без пароля.
   * - ``websockify`` / noVNC
     - TCP 6080
     - HTTP/WebSocket-доступ из браузера.
   * - ``bash``
     - PID основного процесса
     - Запускается после успешной сборки ``sim_ws`` и удерживает контейнер;
       ROS launch пользователь запускает вручную через ``docker exec``.

Entrypoint проверяет запуск каждого фонового процесса. Логи находятся в
``/tmp/xvfb.log``, ``/tmp/fluxbox.log``, ``/tmp/x11vnc.log`` и
``/tmp/novnc.log``.

.. important::

   Успешный ``docker compose up`` означает, что готовы shell и noVNC, но не
   означает, что запущены Gazebo, ROS-ноды или SLAM. Это намеренное поведение
   текущего ``start_sim_novnc.sh``.

Compose
-------

Compose задаёт build context в корне репозитория, контейнер
``rtk2026_sim``, увеличенный ``/dev/shm=1gb`` и публикует noVNC только на
loopback хоста:

.. code-block:: text

   127.0.0.1:6080 -> container:6080

Это важно, потому что VNC работает без пароля. Не заменяйте bind address на
``0.0.0.0`` в недоверенной сети.

Сборка и запуск
---------------

Из корня репозитория:

.. code-block:: bash

   docker compose -f docker/docker-compose.sim.yml up -d --build
   docker compose -f docker/docker-compose.sim.yml ps

Интерфейс noVNC:

.. code-block:: text

   http://127.0.0.1:6080/vnc.html

Вход в контейнер:

.. code-block:: bash

   docker exec -it rtk2026_sim bash

В интерактивном shell ``.bashrc`` подключает ROS 2 Jazzy и собранный
``/workspaces/sim_ws/install``. Запуск полного стека выполняется вручную:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py

Для ``rqt_graph``, teleop и диагностических команд откройте ещё один shell:

.. code-block:: bash

   docker exec -it rtk2026_sim bash

Полный список команд и ожидаемых топиков: :doc:`running`.

Остановка:

.. code-block:: bash

   docker compose -f docker/docker-compose.sim.yml down

Запуск с другим world
---------------------

Launch принимает абсолютный путь аргументом ``world``. Стандартный
установленный мир:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/workspace/install/rtk2026_description/share/rtk2026_description/worlds/rtk2026_slam_world.sdf

Корневой ``worlds/polygon_5x5.world`` копируется в ``/workspace/worlds``:

.. code-block:: bash

   ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/workspace/worlds/polygon_5x5.world

Сохранённая карта для AMCL находится в ``/workspace/maps/my_map.yaml``.
Полный двухпроцессный запуск описан в :doc:`localization`.

Диагностика
-----------

.. code-block:: bash

   docker logs -f rtk2026_sim
   docker exec -it rtk2026_sim bash
   docker exec rtk2026_sim bash -lc 'ps -ef'
   docker exec rtk2026_sim bash -lc \
     'source /opt/ros/jazzy/setup.bash; source /workspaces/sim_ws/install/setup.bash; ros2 node list'
   docker exec rtk2026_sim bash -lc 'tail -100 /tmp/xvfb.log'
   docker exec rtk2026_sim bash -lc 'tail -100 /tmp/novnc.log'

Для software rendering ожидается меньшая частота GUI, чем на нативном GPU.
Физическую частоту проверяют по ``/clock`` и sensor topics, а не по плавности
noVNC-картинки.

Исходники: `docker/ <https://github.com/Hanqnero/RTK2026/tree/main/docker>`_.

Другие Docker-конфигурации проекта
----------------------------------

Помимо симуляции в проекте есть ещё два независимых набора образов:

``docker/dds_check/``
    Минимальный стенд проверки транспорта между Raspberry Pi и ноутбуком.
    Пакетов проекта не содержит: его задача - отделить проблемы сети
    от проблем прикладного кода. См. :doc:`transport_check`.

``pi/docker/``
    Сборка и прошивка Arduino с Raspberry Pi, а также ретранслятор
    serial-порта в TCP для стендовых инструментов настройки моторов.
    См. :doc:`pi/index` и :doc:`arduino/build`.

``pc/docker/``
    Контейнерный вариант инструментов настройки моторов для ноутбука,
    когда не хочется ставить Python-зависимости на хост.
    См. :doc:`pc/index`.
