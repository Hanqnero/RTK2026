ROS 2-драйвер Arduino
=====================

Пакет ``rtk2026_driver`` отделяет три уровня:

``protocol.py``
   Побитовое представление команд и телеметрии.

``transport.py``
   Потокобезопасное неблокирующее чтение и запись Serial.

``arduino_bridge.py``
   ROS-параметры, подписка |cmd_vel|, публикация |odom| и TF.

Поток данных
------------

.. code-block:: text

   /cmd_vel (TwistStamped)
      │ _on_cmd_vel: finite check + saturation
      ▼
   _target_linear_mps / _target_angular_rps
      │ timer 50 Hz + host dead-man
      ▼
   pack_command() -> 9 bytes -> SerialTransport.write()

   SerialTransport.read_available() -> _receive_buffer
      │ pop_telemetry_packet(), по 32 bytes
      ▼
   TelemetryPacket -> /odom + TF odom -> base_footprint

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
     - ``/odom``
     - topic
     - Выход ``Odometry``.
   * - ``odom_frame``
     - ``odom``
     - frame
     - Родитель позы и TF.
   * - ``base_frame``
     - ``base_footprint``
     - frame
     - Дочерний frame одометрии.
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
     - 1.5
     - м/с
     - Saturation ``twist.linear.x``.
   * - ``max_angular_rps``
     - π/2
     - рад/с
     - Saturation ``twist.angular.z``.
   * - ``debug_raw_encoder``
     - false
     - bool
     - Третий байт ``ControlPacket``.

Важное внутреннее состояние
---------------------------

``_last_cmd_time`` использует :func:`time.monotonic`, а не время из
``TwistStamped.header``. Поэтому зависшие симуляционные часы или неверный
timestamp команды не отключают защитную остановку. ``_receive_buffer`` хранит
неполный хвост Serial между вызовами таймера.

Публикация одометрии
--------------------

Arduino уже передаёт интегрированные X, Y и heading. Bridge не пересчитывает
кинематику, а только формирует quaternion:

.. math::

   q_z = \sin(\theta/2), \qquad q_w = \cos(\theta/2)

Одинаковые pose и timestamp используются в ``nav_msgs/Odometry`` и
``TransformStamped``. Это предотвращает внутреннее расхождение между
``/odom`` и ``odom -> base_footprint``.

Ковариация
~~~~~~~~~~

Текущая реализация не заполняет ``pose.covariance`` и ``twist.covariance`` —
в ROS они остаются нулевыми. Перед объединением с IMU через фильтр состояния
необходимо оценить и задать реальные дисперсии энкодерной одометрии.

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
   :private-members: _on_cmd_vel, _send_command, _read_telemetry, _publish_odometry, _handle_serial_error
   :show-inheritance:

.. autofunction:: rtk2026_driver.arduino_bridge.main

API протокола
-------------

.. automodule:: rtk2026_driver.protocol
   :members:
   :undoc-members:
   :show-inheritance:

``COMMAND_STRUCT`` и ``TELEMETRY_STRUCT`` принудительно используют
little-endian и запрещают нативное выравнивание. ``TelemetryPacket`` объявлен
``frozen=True, slots=True``: разобранное измерение нельзя случайно изменить,
и для каждого пакета не создаётся ``__dict__``.

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

Проверка интерфейсов:

.. code-block:: bash

   ros2 topic info -v /cmd_vel
   ros2 topic hz /odom
   ros2 run tf2_ros tf2_echo odom base_footprint

Исходники: `пакет rtk2026_driver <https://github.com/Hanqnero/RTK2026/tree/main/src/rtk2026_driver>`_.
