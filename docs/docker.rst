Контейнер симуляции и noVNC
===========================

Образ предназначен для Apple Silicon и обычного Linux: базовый
``ros:jazzy-ros-base-noble`` имеет ARM64-вариант, а GUI работает через
виртуальный X11 и программный Mesa OpenGL.

Слои Dockerfile
---------------

1. Устанавливается ROS 2 desktop, Gazebo integration, ``gz_ros2_control``,
   контроллеры, ``slam_toolbox`` и ``teleop_twist_keyboard``.
2. Устанавливается графический стек ``Xvfb → fluxbox → x11vnc → websockify``.
3. В ``/workspace/src`` копируется текущий ``src/``.
4. ``rosdep`` разрешает зависимости, кроме внешнего ``sllidar_ros2``.
5. ``colcon`` собирает только description, driver, slam и bringup.
6. Entrypoint поднимает виртуальный экран и выполняет команду из ``CMD``.

``--symlink-install`` полезен внутри build stage, но исходный каталог хоста не
смонтирован в контейнер. После изменения кода образ необходимо пересобрать.

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
   * - ROS launch
     - PID основного процесса
     - Получает сигналы напрямую благодаря ``exec "$@"``.

Entrypoint проверяет запуск каждого фонового процесса. Логи находятся в
``/tmp/xvfb.log``, ``/tmp/fluxbox.log``, ``/tmp/x11vnc.log`` и
``/tmp/novnc.log``.

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

   docker compose -f docker/docker-compose.sim.yml build
   docker compose -f docker/docker-compose.sim.yml up

Интерфейс RViz:

.. code-block:: text

   http://127.0.0.1:6080/vnc.html

Остановка:

.. code-block:: bash

   docker compose -f docker/docker-compose.sim.yml down

Запуск с другим world
---------------------

Compose позволяет заменить CMD:

.. code-block:: bash

   docker compose -f docker/docker-compose.sim.yml run --rm --service-ports sim \
     ros2 launch rtk2026_bringup sim_slam_launch.py \
     world:=/workspace/install/rtk2026_description/share/rtk2026_description/worlds/rtk2026_slam_world.sdf

Корневой ``worlds/polygon_5x5.world`` сейчас не копируется Dockerfile в образ.
Чтобы использовать его без переноса в пакет description, нужен bind mount или
дополнительный ``COPY``. Предпочтительный вариант — установить world как ресурс
``rtk2026_description`` и передавать путь через ament index.

Диагностика
-----------

.. code-block:: bash

   docker logs -f rtk2026_sim
   docker exec -it rtk2026_sim bash
   docker exec rtk2026_sim bash -lc \
     'source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash; ros2 node list'

Для software rendering ожидается меньшая частота GUI, чем на нативном GPU.
Физическую частоту проверяют по ``/clock`` и sensor topics, а не по плавности
noVNC-картинки.

Исходники: `docker/ <https://github.com/Hanqnero/RTK2026/tree/main/docker>`_.
