Ноутбук
========

Здесь запускаются инструменты диагностики и настройки: подключаются к плате
либо напрямую по USB, либо по сети к ``link_server.py`` на Raspberry Pi
(:doc:`../pi/index`). Общая схема стенда - в :doc:`../bench`.

Код лежит в ``pc/``: ``pc/tools/`` - сами инструменты (описаны в
:doc:`tools`), ``pc/docker/`` - контейнерный вариант для тех, кто не хочет
ставить зависимости на хост.

Локальный запуск
-----------------

Окна открываются сразу, настраивать ничего не нужно.

.. code-block:: bash

   cd pc
   python3 -m venv .venv
   .venv/bin/pip install -r requirement.txt

   export RTK_LINK=192.168.1.50:5555

   .venv/bin/python tools/check_encoders.py --port $RTK_LINK --drive
   .venv/bin/python tools/identify_wheel.py --port $RTK_LINK --wheel both --apply --save --plot
   .venv/bin/python tools/tune_wheel.py     --port $RTK_LINK --wheel left --plot --log records/tune.csv
   .venv/bin/python tools/monitor.py        --port $RTK_LINK --drive
   .venv/bin/python tools/route_test.py     --port $RTK_LINK --pattern figure8 --log records/route.csv

``records/`` игнорируется git-ом: логи стенда - это данные конкретного
прогона, а не код.

В контейнере
-------------

Образ содержит стендовые Python-инструменты, ROS 2 Jazzy, RViz и
RQt. На Linux сервис использует host network и host IPC, поэтому
попадает в тот же DDS-граф ``ROS_DOMAIN_ID=0``, что и ROS-контейнеры
робота. Запускайте compose из графической сессии, не из ``sudo``:

.. code-block:: bash

   export RTK_LINK=192.168.1.50:5555
   export DISPLAY
   export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

   docker compose -f pc/docker/docker-compose.tools.yml build tools
   docker compose -f pc/docker/docker-compose.tools.yml run --rm tools \
       python3 tools/check_encoders.py --port $RTK_LINK --auto --no-plot --log records/enc.csv

Пока работает ROS-стек на Pi, GUI запускается отдельными
одноразовыми контейнерами:

.. code-block:: bash

   docker compose -f pc/docker/docker-compose.tools.yml run --rm tools \
       rviz2 -d /work/rviz/rtk2026_real_robot.rviz
   docker compose -f pc/docker/docker-compose.tools.yml run --rm tools rqt
   docker compose -f pc/docker/docker-compose.tools.yml run --rm tools rqt_graph

Если Qt пишет ``could not connect to display``, проверьте, что
``DISPLAY`` не пуст и файл ``XAUTHORITY`` существует и читается.
В Wayland/Xwayland это обычно файл в ``/run/user/<uid>/``, а не
``~/.Xauthority``; compose монтирует значение переменной.

Логи пишутся в ``records/`` смонтированного дерева и остаются на хосте.

Окна из контейнера на macOS
-----------------------------

Требуется XQuartz. Вариант рабочий, но настраивается руками, и ради графиков
проще пользоваться локальным ``.venv``.

.. code-block:: bash

   brew install --cask xquartz
   open -a XQuartz
   # XQuartz -> Settings -> Security -> Allow connections from network clients
   xhost + 127.0.0.1

   export DISPLAY=host.docker.internal:0
   docker compose -f pc/docker/docker-compose.tools.yml run --rm tools \
       python3 tools/monitor.py --port $RTK_LINK --drive

``network_mode: host`` в Docker Desktop на macOS не даёт контейнеру
настоящий L2/multicast-доступ к сети Pi. XQuartz решает только
вывод окон; для ROS-топиков используйте DDS Router и ``viz``
из :doc:`../transport_check`.

Дальше
------

Полное описание каждого инструмента и порядок настройки привода -
в :doc:`tools`.

.. toctree::
   :hidden:

   tools
