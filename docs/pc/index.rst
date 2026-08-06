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

Нужен, если не хочется ставить зависимости на хост. Окна Qt при этом
не работают без дополнительной настройки, поэтому вариант годится для
режимов ``--text`` и ``--no-plot`` и для сбора логов.

.. code-block:: bash

   export RTK_LINK=192.168.1.50:5555

   docker compose -f pc/docker/docker-compose.tools.yml run --rm tools \
       python3 tools/check_encoders.py --port $RTK_LINK --auto --no-plot --log records/enc.csv

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

Дальше
------

Полное описание каждого инструмента и порядок настройки привода -
в :doc:`tools`.

.. toctree::
   :hidden:

   tools
