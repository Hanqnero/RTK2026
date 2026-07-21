Сборка и прошивка Arduino
=========================

Сборка использует CMake ≥ 3.23, Ninja и AVR GCC. Целевая платформа:
``atmega2560``, частота ``F_CPU=16 MHz``.

Варианты firmware
-----------------

.. list-table:: CMake options
   :header-rows: 1

   * - Опция
     - Исходники
     - Назначение
   * - обе ``OFF``
     - ``main.cpp``, ``encoder.cpp``, ``sonar.cpp``
     - Основной PID-контур.
   * - ``ROBOT_DIRECT_MOTOR_TEST=ON``
     - ``main_direct_motor_test.cpp``, ``encoder.cpp``
     - Проверка PWM и знаков энкодеров.
   * - ``ROBOT_BMI270_TEST=ON``
     - ``main_bmi270_test.cpp``, ``bmi270_i2c.cpp``, Wire
     - Изолированная проверка IMU.

Одновременно включить оба тестовых варианта нельзя.

Пресеты
-------

``windows-bundled``
   Использует включённый в проект Windows AVR toolchain.

``macos-system``
   Использует системный AVR GCC; пути Homebrew вынесены в cache variables.

``linux-system``
   Использует ``toolchains/avr-system.cmake`` и AVR-пакеты системы.

Пример macOS/Linux:

.. code-block:: bash

   cd arduino
   cmake --preset macos-system
   cmake --build build

Результат: ``arduino/build/robot_control_interface.hex``. После линковки
``avr-size`` показывает использование flash/RAM.

Прошивка
--------

Если CMake нашёл ``avrdude``:

.. code-block:: bash

   cmake --build build --target flash

Порт задаётся на этапе конфигурации:

.. code-block:: bash

   cmake --preset macos-system \
     -DUPLOAD_PORT=/dev/cu.usbmodem14101

Параметры по умолчанию: protocol ``wiring``, baud 115200, MCU
``atmega2560``. Target ``flash`` зависит от ELF и поэтому всегда сначала
пересобирает firmware.
