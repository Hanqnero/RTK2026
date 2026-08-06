ROS 2-драйвер Arduino
=====================

Пакет ``rtk2026_driver`` отделяет три уровня:

``protocol.py``
   Побитовое представление команд и телеметрии.

``transport.py``
   Потокобезопасное неблокирующее чтение и запись Serial.

``arduino_bridge.py``
   ROS-параметры, подписка |cmd_vel| и публикация сырой |wheel_odom|.

Поток данных
------------

.. code-block:: text

   /cmd_vel (TwistStamped)
      │ _on_cmd_vel: finite check + saturation
      ▼
   _target_linear_mps / _target_angular_rps
      │ timer 50 Hz + host dead-man
      ▼
   pack_velocity_command() -> кадр 15 байт -> SerialTransport.write()

   SerialTransport.read_available() -> FrameDecoder.feed()
      │ проверка CRC, восстановление синхронизации
      ├── MSG_TELEMETRY -> decode_telemetry() -> SequenceTracker
      │      ▼
      │   TelemetryPacket -> /wheel/odom -> EKF
      │                              ├── /odometry/filtered
      │                              └── TF odom -> base_footprint
      └── MSG_STATS -> decode_stats() -> _latest_stats -> лог линка

Параметры ArduinoBridgeNode
---------------------------

Параметры объявляются в конструкторе и получают значения из
``config/arduino_bridge.yaml``.

.. list-table:: Параметры ноды ``arduino_bridge``
   :header-rows: 1
   :widths: 32 18 18 32

   * - Имя
     - Default
     - Единица/тип
     - Назначение
   * - ``serial_port``
     - ``/dev/arduino``
     - path
     - Устройство, проброшенное в контейнер.
   * - ``baudrate``
     - 115200
     - бод
     - Должно совпадать с ``kSerialBaudRate``.
   * - ``arduino_reset_wait_sec``
     - 1.0
     - с
     - Пауза после DTR-reset Arduino.
   * - ``cmd_vel_topic``
     - ``/cmd_vel``
     - topic
     - Вход ``TwistStamped``.
   * - ``odom_topic``
     - ``/wheel/odom``
     - topic
     - Сырой выход ``Odometry`` по энкодерам.
   * - ``odom_frame``
     - ``odom``
     - frame
     - Родитель позы и TF.
   * - ``base_frame``
     - ``base_footprint``
     - frame
     - Дочерний frame одометрии.
   * - ``publish_odom_tf``
     - false
     - bool
     - Аварийный TF без EKF; в штатном bringup должен оставаться false.
   * - ``pose_covariance_diagonal``
     - 6 чисел
     - м²/рад²
     - Диагональ неопределённости позы.
   * - ``twist_covariance_diagonal``
     - 6 чисел
     - (м/с)²/(рад/с)²
     - Диагональ неопределённости скоростей.
   * - ``command_send_interval_sec``
     - 0.02
     - с
     - 50 Гц повторной отправки последней команды.
   * - ``telemetry_poll_period_sec``
     - 0.01
     - с
     - 100 Гц опроса входного буфера; это не частота firmware.
   * - ``drop_stale_cmd_after_sec``
     - 0.30
     - с
     - Host dead-man, после которого передаются нули.
   * - ``max_linear_mps``
     - 1.69
     - м/с
     - Saturation ``twist.linear.x``. Выводится из паспортных оборотов
       мотора, радиуса колеса и запаса 30 %; обязано совпадать
       с ``kMaxLinearCommandMps`` прошивки.
   * - ``max_angular_rps``
     - π/2
     - рад/с
     - Saturation ``twist.angular.z``.
   * - ``link_report_period_sec``
     - 10.0
     - с
     - Период отчёта о качестве линка в лог.

Параметр ``debug_raw_encoder`` удалён вместе с протоколом v1: в v2 дельты
энкодеров передаются всегда, переключать их больше нечем.

Важное внутреннее состояние
---------------------------

``_last_cmd_time`` использует :func:`time.monotonic`, а не время из
``TwistStamped.header``. Поэтому зависшие симуляционные часы или неверный
timestamp команды не отключают защитную остановку.

``_decoder`` хранит неполный хвост кадра между вызовами таймера и считает
ошибки CRC и мусорные байты. ``_sequence`` отслеживает разрывы в поле ``seq``
и отличает потерю пакета от задержки. ``_latest_stats`` держит последний
``StatsPacket`` прошивки.

Отчёт о состоянии линка
-----------------------

Раз в ``link_report_period_sec`` нода пишет в лог число принятых и потерянных
пакетов, ошибки CRC и счётчики самой прошивки: фактический период цикла,
срывы периода, потери на передаче и свободную RAM.

Уровень INFO, если отклонений нет, и WARN, если были потери, повреждения
потока или срывы периода. Эти события ничем другим себя не проявляют:
без отчёта потеря синхронизации выглядит просто как более редкая телеметрия.

Публикация одометрии
--------------------

Arduino уже передаёт интегрированные X, Y и heading. Bridge не пересчитывает
кинематику, а только формирует quaternion:

.. math::

   q_z = \sin(\theta/2), \qquad q_w = \cos(\theta/2)

Pose и twist получают один timestamp и публикуются как
``nav_msgs/msg/Odometry`` в ``/wheel/odom``. В штатном режиме bridge не
публикует TF: ``odom -> base_footprint`` формирует EKF по этому измерению.
Параметр ``publish_odom_tf=true`` оставлен только для изолированной проверки
bridge без EKF.

Ковариация
~~~~~~~~~~

Bridge заполняет диагонали ``pose.covariance`` и ``twist.covariance`` из
YAML. Стартовые значения выражают меньшую уверенность по Z/roll/pitch и
конечную неопределённость планарной одометрии. Это не автоматическая
калибровка: значения X/Y/yaw и vx/vy/vyaw нужно уточнить по повторяемым
экспериментам.

Обработка ошибок и завершение
-----------------------------

Ошибка чтения или записи Serial логируется как ``fatal``, порт закрывается,
затем вызывается ``rclpy.shutdown``. Автоматического переподключения сейчас
нет. При штатном ``destroy_node`` bridge сначала пытается отправить нулевую
команду, затем закрывает порт.

API ноды
--------

.. autoclass:: rtk2026_driver.arduino_bridge.ArduinoBridgeNode
   :members: destroy_node
   :private-members: _read_covariance_diagonal, _on_cmd_vel, _send_command, _read_telemetry, _publish_odometry, _handle_serial_error
   :show-inheritance:

.. autofunction:: rtk2026_driver.arduino_bridge.main

API протокола
-------------

.. automodule:: rtk2026_driver.protocol
   :members:
   :undoc-members:
   :show-inheritance:

``VELOCITY_STRUCT``, ``TELEMETRY_STRUCT`` и ``STATS_STRUCT`` принудительно
используют little-endian и запрещают нативное выравнивание. ``TelemetryPacket``
объявлен ``frozen=True, slots=True``: разобранное измерение нельзя случайно
изменить, и для каждого пакета не создаётся ``__dict__``.

Модуль дублирует стендовый кодек ``protocol/rtk_link.py``. Дублирование
намеренное: стендовые скрипты обязаны работать без установленного ROS.
Тест ``test_ros_and_bench_codecs_agree`` сверяет обе копии байт в байт.

API транспорта
--------------

.. autoclass:: rtk2026_driver.transport.SerialTransport
   :members:
   :show-inheritance:

``timeout=0`` делает чтение неблокирующим. ``write_timeout_sec`` ограничивает
зависание записи. Один ``threading.Lock`` защищает состояние порта и позволяет
в дальнейшем использовать транспорт с ``MultiThreadedExecutor``.

Запуск
------

.. code-block:: bash

   ros2 launch rtk2026_bringup arduino_launch.py

Нода запускается на Raspberry Pi: она владеет serial-портом Arduino.
На компьютере разработчика её запускать незачем, там нет платы.

.. warning::

   Serial-порт держит только один процесс. За устройство ``/dev/arduino``
   конкурируют три вещи:

   * ``arduino_bridge`` — эта нода;
   * ``link_server.py`` — ретранслятор стенда настройки моторов,
     см. :doc:`bench`;
   * ``avrdude`` при прошивке, см. :doc:`arduino/build`.

   Одновременно работать они не будут. Перед настройкой моторов остановите
   ROS-стек, перед поездкой под ROS остановите ретранслятор.

Проверка интерфейсов:

.. code-block:: bash

   ros2 topic info -v /cmd_vel
   ros2 topic hz /wheel/odom
   ros2 topic echo /wheel/odom --once --field twist.covariance
   ros2 run tf2_ros tf2_echo odom base_footprint

Исходники: `пакет rtk2026_driver <https://github.com/Hanqnero/RTK2026/tree/main/src/rtk2026_driver>`_.
