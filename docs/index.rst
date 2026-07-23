RTK2026: техническая документация
=================================

Документация описывает отработанную часть RTK2026: прошивку Arduino Mega,
ROS 2-драйвер, построение карты, модели URDF/Xacro, сценарии запуска и
контейнер симуляции. Значения параметров и интерфейсы приведены по текущему
состоянию исходников, а не как общий учебник по ROS.

.. important::

   Единственный внешний интерфейс управления движением — |cmd_vel| типа
   ``geometry_msgs/msg/TwistStamped``. В симуляции команду принимает
   ``diff_drive_controller``, на реальном роботе — ``ArduinoBridgeNode``.

.. toctree::
   :maxdepth: 2
   :caption: Система

   architecture
   interfaces

.. toctree::
   :maxdepth: 3
   :caption: Модули

   arduino/index
   driver
   localization
   slam
   description/index
   bringup
   docker

.. toctree::
   :maxdepth: 2
   :caption: Эксплуатация и разработка

   running
   calibration
   diagnostics
   development

Границы документации
--------------------

В этот выпуск намеренно не включены компьютерное зрение, маршрутизация,
манипулятор и экспериментальные пакеты. Справочник фиксирует только модули,
перечисленные выше, и их непосредственные внешние зависимости.

Индексы
-------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
