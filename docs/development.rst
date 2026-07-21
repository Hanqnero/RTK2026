Сборка и сопровождение документации
===================================

Локальная сборка
----------------

Sphinx не требует установленного ROS: ROS-модули mock-ируются в ``conf.py``, а
чистые Python-модули протокола импортируются из рабочего дерева.

.. code-block:: bash

   python3 -m pip install -r docs/requirements.txt
   make -C docs html

Результат:

.. code-block:: text

   docs/_build/html/index.html

Строгая сборка использует ``-W --keep-going``: любое предупреждение о
неизвестной директиве, дублированной цели или битой внутренней ссылке считается
ошибкой.

Проверка внешних ссылок
-----------------------

.. code-block:: bash

   make -C docs linkcheck

Результаты находятся в ``docs/_build/linkcheck/output.txt``. Сетевые ошибки
следует отличать от реального HTTP 404.

Как обновлять документацию
--------------------------

При изменении интерфейса обновите связанные страницы:

.. list-table:: Изменение и обязательная документация
   :header-rows: 1

   * - Что изменилось
     - Где обновить
   * - Поля/размер Serial packet
     - ``arduino/protocol.rst``, ``rtk2026_driver.protocol``, firmware tests.
   * - Pin или флаг reverse
     - ``arduino/hardware.rst`` и ``arduino/pinout.md``.
   * - Радиус/колея
     - Xacro properties, controller YAML, ``description/models.rst``.
   * - ROS topic/frame
     - ``interfaces.rst``, driver, bringup, SLAM config и RViz.
   * - Xacro macro parameter
     - ``description/xacro_reference.rst`` и все места вызова макроса.
   * - Launch argument
     - ``bringup.rst`` и команды Docker.
   * - Docker package или port
     - ``docker.rst`` и Compose/Dockerfile comments.

Ссылки на исходники
-------------------

Python API использует ``linkcode`` и ведёт на соответствующие строки GitHub.
По умолчанию используется branch ``main``; для release/tag задайте:

.. code-block:: bash

   RTK2026_DOCS_SOURCE_REF=v0.1.0 make -C docs html

Репозиторий: `Hanqnero/RTK2026 <https://github.com/Hanqnero/RTK2026>`_.
