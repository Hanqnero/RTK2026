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

.. warning::

   Пресет ``macos-system`` жёстко указывает на AVR GCC из Homebrew
   (``/opt/homebrew/Cellar/avr-gcc@9/9.5.0``). Если этой установки нет,
   конфигурация упадёт.

   Рабочая альтернатива без Homebrew — тулчейн из состава ``arduino-cli``.
   Он ставится вместе с ядром ``arduino:avr`` и лежит в
   ``~/Library/Arduino15/packages/arduino/tools/avr-gcc/``:

   .. code-block:: bash

      AVR=~/Library/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7
      export PATH="$AVR/bin:$PATH"

      cd arduino
      cmake -S . -B build -G Ninja \
          -DCMAKE_TOOLCHAIN_FILE=toolchains/avr-system.cmake \
          -DAVR_TOOLCHAIN_ROOT="$AVR" \
          -DAVR_STDLIB_INCLUDE_DIR="$AVR/avr/include"

      cmake --build build

   Предупреждения вида ``plugin needed to handle lto object`` от ``avr-ar``
   и ``avr-ranlib`` при этом нормальны: LTO включён, а обёртки над ними
   в этом тулчейне плагин не подхватывают. На линковку это не влияет.

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

Прошивка с Raspberry Pi
-----------------------

На роботе плата подключена к Raspberry Pi, а не к компьютеру разработчика,
поэтому прошивка выполняется оттуда. Тулчейн и ``avrdude`` живут в контейнере,
ставить их на саму Pi не требуется.

.. code-block:: bash

   cd ~/RTK2026

   # только сборка, устройство не нужно
   docker compose -f pi/docker/docker-compose.pi.yml run --rm build

   # сборка и заливка
   docker compose -f pi/docker/docker-compose.pi.yml run --rm flash

Каталог сборки ``build-pi/`` отделён от локального ``build/``, чтобы кэш
кросс-компиляции под ARM не смешивался с кэшем компьютера разработчика.
Образ собирается из корня репозитория: контейнеру нужен доступ и
к прошивке в ``arduino/``, и к ретранслятору в ``pi/tools/``. Подробнее
в :doc:`../pi/index`.

.. important::

   Serial-порт держит только один процесс. Перед прошивкой остановите
   ретранслятор ``link_server`` и ROS-ноду ``arduino_bridge``: оба
   обращаются к тому же устройству. Подробнее в :doc:`../bench`.
