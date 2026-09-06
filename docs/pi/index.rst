Raspberry Pi
=============

Бортовой компьютер робота. Держит serial-порты Arduino и лидара, собирает
и заливает прошивку, крутит ROS-стек с картографированием и ретранслирует
топики на ноутбук. Общая схема стенда - в :doc:`../bench`, транспорт до
ноутбука - в :doc:`../transport_check`.

Что на ней лежит
----------------

На робота едет не весь репозиторий, а только исполняемая им часть.
Синхронизация одной командой с ноутбука:

.. code-block:: bash

   pi/tools/sync_to_pi.sh              # обычная синхронизация
   CLEAN=1 pi/tools/sync_to_pi.sh      # ещё и убрать лишнее

Обычный deploy кода сразу синхронизирует дерево и пересоздаёт ROS-контейнер,
явно запретив Docker собирать image:

.. code-block:: bash

   pi/tools/deploy_to_pi.sh

Системные зависимости находятся в image, а исходники подключены bind mount.
Поэтому image нужно пересобирать только после изменения ``Dockerfile.ros``,
``package.xml``, startup-скрипта или vendor-драйвера:

.. code-block:: bash

   REBUILD_IMAGES=1 pi/tools/deploy_to_pi.sh

Для точки доступа робота передайте адрес первым аргументом. Набор сервисов
задаётся через ``SERVICES``:

.. code-block:: bash

   pi/tools/deploy_to_pi.sh 10.42.0.1
   SERVICES="ros perception" pi/tools/deploy_to_pi.sh

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Каталог
     - Назначение
   * - ``arduino/``
     - Прошивка, ядро AVR, vendor-библиотеки.
   * - ``pi/``
     - Docker-конфигурации и ``link_server.py``.
   * - ``protocol/``
     - Общий кодек протокола, используют и стенд, и ретранслятор.
   * - ``docker/dds_check/``
     - DDS Router на стороне робота.
   * - ``src/``
     - Пакеты основного ROS-стека и ``rtk2026_cv`` с ONNX-моделью для
       отдельного perception-контейнера.
   * - ``maps/``, ``records/``
     - Рабочие каталоги робота: карты и записи прогонов.

``maps/`` и ``records/`` скрипт только создаёт и никогда не удаляет: там
данные, которых нет в репозитории. Всё остальное зеркалируется с
``--delete``, поэтому лишний файл на роботе не переживёт синхронизации.

Образ не копирует эти исходники. Compose собирает
``/workspaces/robot_ws/src`` из bind mounts нужных Pi пакетов.
``build/``, ``install/`` и ``log/`` живут в ``workspaces/robot_ws``
на Pi и сохраняются при пересоздании контейнера. Vendor-драйверы
собраны отдельно в ``/opt/vendor_ws`` внутри образа.

Доступ
------

Два независимых пути, оба работают одновременно.

.. list-table::
   :header-rows: 1
   :widths: 18 22 60

   * - Путь
     - Адрес
     - Когда
   * - Ethernet
     - ``pi.local``
     - Основной для разработки. Через него же идут ROS-топики на ноутбук.
   * - Wi-Fi робота
     - ``10.42.0.1``
     - Точка ``ROSSIYANE``, поднимается на USB-свистке автоматически.
       Сеть без интернета, только для связи с роботом.

.. code-block:: bash

   ssh pi@pi.local

Часы синхронизируются по NTP при наличии интернета. Это существенно:
RViz на ноутбуке считает возраст трансформаций по своим часам, и
расхождение даёт ``extrapolation into the future`` - пустую сцену без
внятной ошибки. Проверить:

.. code-block:: bash

   timedatectl | grep synchronized

Контейнеры
----------

Все объявлены в ``pi/docker/docker-compose.pi.yml``.

.. list-table::
   :header-rows: 1
   :widths: 16 84

   * - Сервис
     - Назначение
   * - ``ros``
     - ROS-стек робота: привод, лидар, EKF, SLAM. Основной рабочий.
   * - ``link``
     - Ретранслятор serial в TCP для стендовых инструментов настройки
       моторов.
   * - ``flash``
     - Сборка и заливка прошивки Arduino.
   * - ``build``
     - Сборка прошивки без заливки.
   * - ``shell``
     - Оболочка с ``avrdude`` для ручных операций.
   * - ``camera``
     - Только USB-камера, для записи и стендовой диагностики.
   * - ``perception``
     - USB-камера и ONNX YOLO-детектор знаков. Штатный вариант для
       автономного прогона.

.. important::

   Serial-порт держит ровно один процесс. ``link``, ``flash`` и нода
   ``arduino_bridge`` внутри ``ros`` конкурируют за ``/dev/arduino``.
   Перед прошивкой останавливайте остальных.

.. warning::

   **После каждого переподключения USB пересоздавайте контейнер.**

   Docker фиксирует устройство по номерам major:minor в момент создания
   контейнера. Если ``ttyUSB0`` и ``ttyUSB1`` поменялись местами - а это
   происходит при любом переподключении, - внутри контейнера
   ``/dev/rplidar`` будет указывать на Arduino, и лидар молча перестанет
   отвечать.

   .. code-block:: bash

      docker compose -f pi/docker/docker-compose.pi.yml up -d --no-build --force-recreate ros

Устройства
----------

Пробрасываются по идентификатору, а не по ``/dev/ttyUSB0``: номера
присваиваются в порядке подключения и меняются, идентификатор постоянен.

.. code-block:: bash

   ls -l /dev/serial/by-id/

.. list-table::
   :header-rows: 1
   :widths: 22 26 52

   * - Устройство
     - Мост
     - Путь в контейнере
   * - Arduino Mega
     - CH340 (``1a86:7523``)
     - ``/dev/arduino``
   * - RPLIDAR A1M8
     - CP2102 (``10c4:ea60``)
     - ``/dev/rplidar``

Прошить Arduino
---------------

.. code-block:: bash

   cd ~/RTK2026
   docker compose -f pi/docker/docker-compose.pi.yml stop link
   docker compose -f pi/docker/docker-compose.pi.yml run --rm flash

Только сборка, без заливки:

.. code-block:: bash

   docker compose -f pi/docker/docker-compose.pi.yml run --rm build

Каталог сборки ``arduino/build-pi/`` отделён от локального
``arduino/build/`` и не синхронизируется с ноутбука: это кэш
кросс-компиляции самой Pi.

Запустить ROS-стек
------------------

.. code-block:: bash

   pi/tools/deploy_to_pi.sh

Перед запуском Bash контейнер сам выполняет ``colcon build
--symlink-install`` для пакетов робота. Лог сборки виден через
``docker logs rtk2026-ros``.

``tmux`` и ``vim`` установлены в образ для работы в интерактивной
сессии на роботе.

Окружение ROS подключается в ``.bashrc``, поэтому ``docker exec -it``
его видит, а ``docker exec bash -lc`` - нет: login-оболочка ``.bashrc``
не читает. В скриптах подключайте явно:

.. code-block:: bash

   docker exec -it rtk2026-ros bash -c '
     source /opt/ros/jazzy/setup.bash
     source /opt/vendor_ws/install/setup.bash
     source /workspaces/robot_ws/install/setup.bash
     ros2 launch rtk2026_bringup full.launch.py'

Порядок подключения важен: underlay с драйвером лидара идёт до overlay
рабочего пространства, иначе ``lidar_launch.py`` не найдёт
``sllidar_ros2``.

Если образ был собран старой версией Dockerfile и ``ros2 pkg prefix
sllidar_ros2`` всё ещё не находит пакет, один раз пересоберите слой драйвера
без кэша:

.. code-block:: bash

   docker compose -f pi/docker/docker-compose.pi.yml build --no-cache ros
   docker compose -f pi/docker/docker-compose.pi.yml up -d --force-recreate ros

Отдельные части стека:

.. code-block:: bash

   ros2 launch rtk2026_bringup lidar_launch.py     # только лидар
   ros2 launch rtk2026_bringup arduino_launch.py   # только привод
   ros2 launch rtk2026_bringup full.launch.py       # весь аппаратный стек
   ros2 launch rtk2026_bringup real_slam.py         # аппаратура, EKF и SLAM

Автономный прогон запускается в трёх терминалах контейнера. После первой
команды задайте начальную позу в RViz:

.. code-block:: bash

   ros2 launch rtk2026_bringup real_localization.py \
     map:=/workspaces/robot_ws/maps/my_map.yaml
   ros2 launch rtk2026_nav2 nav2.launch.py use_sim_time:=false
   ros2 launch rtk2026_city_nav city_nav.launch.py \
     use_sim_time:=false \
     graph_path:=/workspaces/robot_ws/maps/graph \
     start_previous_vertex:=<PREVIOUS_ID> \
     start_current_vertex:=<CURRENT_ID>

Распознавание знаков
--------------------

В автономном прогоне камера и детектор запускаются одним сервисом. Он
намеренно отделён от ``ros``: отказ камеры не останавливает привод, лидар и
локализацию.

.. code-block:: bash

   docker compose -f pi/docker/docker-compose.pi.yml stop camera
   SERVICES=perception pi/tools/deploy_to_pi.sh
   docker logs -f rtk2026-perception

``camera`` и ``perception`` одновременно запускать нельзя: оба открывают
``/dev/video0``. Детектор публикует только компактный
``/perception/driving_detection``; исходный поток кадров остаётся на Pi 5.

Проверка:

.. code-block:: bash

   docker exec -it rtk2026-perception bash -c '
     source /opt/ros/jazzy/setup.bash
     source /workspace/install/setup.bash
     ros2 topic hz /camera/image_raw'
   docker exec -it rtk2026-ros bash -c '
     source /opt/ros/jazzy/setup.bash
     source /workspaces/robot_ws/install/setup.bash
     ros2 topic echo /perception/driving_detection --once'

В логе детектора печатаются время инференса и возраст обработанного кадра.
Если возраст растёт, сначала оставьте ``input_size: 416`` и уменьшите число
``intra_op_num_threads`` в ``sign_detection_pi5.yaml``. Площадь рамки для
``city_nav.min_box_area_px`` калибруется на реальной дистанции принятия
решения; до этого нулевое значение безопасно оставляет знаки выключенными.

Лидар
-----

На роботе установлен RPLIDAR C1. Драйвер ``sllidar_ros2`` подключён
к репозиторию как Git-подмодуль и зафиксирован на проверенном
commit. После клонирования инициализируйте его:

.. code-block:: bash

   git submodule update --init vendor/sllidar_ros2

Профиль C1 по умолчанию использует 460800 бод и режим ``Standard``.
Ниже сохранены замеры запасного A1M8 (прошивка 1.29); все режимы
с дальностью 12 м:

.. list-table::
   :header-rows: 1
   :widths: 10 22 20 48

   * - id
     - Режим
     - Выборка
     - Точек за оборот при 6.5 Гц
   * - 0
     - Standard
     - 2.0 кГц
     - около 310
   * - 1
     - Express
     - 3.9 кГц
     - около 600
   * - 2
     - Boost
     - 7.9 кГц
     - около 1200
   * - 3
     - Sensitivity
     - 7.9 кГц
     - около 1200, выбран в ``lidar_launch.py``
   * - 4
     - Stability
     - 5.0 кГц
     - около 770

У запасного A1M8 частоту вращения драйвер не настраивает: её задаёт PWM мотора,
фактическая около 6.5 Гц. За один оборот при 0.25 м/с робот проезжает
38 мм, а ``slam_toolbox`` скан не расшивает - поэтому карту стоит строить
на пониженной скорости, около 0.15 м/с и не выше 0.6 рад/с по углу.

Проверка связи с лидаром без ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Отделяет неисправность канала от проблем драйвера. Ответ в 27 байт,
начинающийся с ``a5 5a``, означает исправную связь; ноль байт - обрыв.

.. code-block:: bash

   docker exec rtk2026-ros python3 -c "
   import serial,time
   p=serial.Serial('/dev/rplidar',115200,timeout=2); p.dtr=False; time.sleep(0.5)
   p.write(b'\xA5\x25'); time.sleep(0.3); p.write(b'\xA5\x50'); time.sleep(1)
   d=p.read(64); print(len(d),'байт', d[:8].hex(' '))"

Запускать серией из восьми: единичный успех ничего не доказывает.
Запуск сканирования - цепочка из четырёх обменов, и при 75 % успеха на
шаг она проходит лишь в трети случаев. Симптом при этом выглядит как
отказ драйвера, хотя виноват канал.

.. note::

   Не запускайте эту проверку одновременно с драйвером. Несколько
   процессов могут открыть один tty, и байты достаются тому, кто успел
   прочитать первым - оба будут работать через раз.

Транспорт до ноутбука
---------------------

ROS-топики уходят на ноутбук через DDS Router: между машинами нет
надёжного multicast, и Router заменяет автообнаружение одним явным
TCP-соединением. Подробности и разбор типичных отказов - в
:doc:`../transport_check`.

На роботе:

.. code-block:: bash

   cd ~/RTK2026/docker/dds_check
   VULCANEXUS_TAG=jazzy-cloud PI_ADDRESS=192.168.2.2 \
     docker compose -f docker-compose.pi.yml up -d router

``VULCANEXUS_TAG=jazzy-cloud`` обязателен: в базовом варианте образа
пакета ``vulcanexus-jazzy-ddsrouter`` нет, и сборка падает с
``Unable to locate package``.

``PI_ADDRESS`` обязан совпадать с тем, который набирает ноутбук.
Fast DDS сопоставляет объявленный локатор с открытым TCP-соединением, и
при несовпадении данные не идут, хотя соединение установлено, а топики
на той стороне видны.

На ноутбуке:

.. code-block:: bash

   cd docker/dds_check
   export PI_ADDRESS=192.168.2.2
   export VULCANEXUS_TAG=jazzy-cloud
   docker compose -f docker-compose.mac.yml up -d viz

RViz открывается в браузере: ``http://127.0.0.1:6080/vnc.html``

Контейнер ``viz`` держит RViz и Router вместе намеренно: тогда
виртуализированную сеть Docker Desktop пересекает ровно одно исходящее
TCP-соединение, а между Router и RViz работает обычный локальный DDS.

.. note::

   ``Fixed Frame`` в RViz должен существовать в дереве TF. Когда запущен
   только лидар, единственный фрейм - ``lidar_frame``; ``map`` и ``odom``
   появляются лишь вместе с ``real_slam.py``. С несуществующим фиксированным
   фреймом RViz не рисует ничего, хотя ``Status`` у дисплея остаётся ``Ok``.

Проверки
--------

.. code-block:: bash

   # телеметрия от Arduino
   docker exec rtk2026-ros bash -c 'source /opt/ros/jazzy/setup.bash; ros2 topic hz /wheel/odom'

   # скан лидара
   docker exec rtk2026-ros bash -c 'source /opt/ros/jazzy/setup.bash; ros2 topic hz /scan'

   # число лучей в скане
   docker exec rtk2026-ros bash -c 'source /opt/ros/jazzy/setup.bash;
     ros2 topic echo /scan --once --field ranges | tr "," "\n" | grep -c .'

   # дерево TF
   docker exec rtk2026-ros bash -c 'source /opt/ros/jazzy/setup.bash; ros2 run tf2_tools view_frames'

   # питание и просадки: 0x0 - норма, 0x10000 - просадка была
   vcgencmd get_throttled
   vcgencmd pmic_read_adc | grep EXT5V_V

Диагностика ROS-стека запускается профилем реального робота:

.. code-block:: bash

   ros2 launch rtk2026_observability diagnostics_real.launch.py

Профиль ``real`` отличается от симуляционного составом ожиданий: на
роботе нет Gazebo-мостов и ground truth, а частоты заданы железом.
Мониторы опираются на ROS graph и работают только на самом роботе: через
DDS Router граф не проходит достоверно.

Питание
-------

``usb_max_current_enable=1`` в ``/boot/firmware/config.txt`` снимает
ограничение суммарного тока USB с 600 мА до 1.6 А. Без него три
устройства - лидар, Arduino и Wi-Fi-свисток - лимит выбирают целиком, и
сканирующее ядро лидара не стартует, хотя мотор крутится.

.. warning::

   Флаг требует блока питания на 5 А (27 Вт). С более слабым Pi уходит
   в защиту под нагрузкой: гаснет периферия, горит красный индикатор.
   Альтернатива без флага - активный USB-хаб с внешним питанием.

``link_server.py``
-------------------

Ретранслятор serial-порта в TCP. Ничего не разбирает и не переупаковывает:
не знает ни про кадры, ни про CRC. Поэтому кадрирование и контрольные суммы
работают из конца в конец, от прошивки до инструмента на ноутбуке, и
повреждение по дороге будет замечено кодеком там, а не спрятано
ретранслятором.

Телеметрия рассылается всем подключённым клиентам, поэтому панель состояния
и инструмент настройки на ноутбуке могут работать одновременно. Команды же
принимаются от всех клиентов и сливаются в один поток: два одновременно
командующих инструмента будут мешать друг другу, прошивка исполнит ту
команду, что пришла последней. Сервер это не запрещает - он не может
отличить осмысленную одновременную работу от ошибки оператора, - но считает
число писавших клиентов и печатает его в статистике.

.. automodule:: link_server
   :members:
   :undoc-members:
   :show-inheritance:
