# Работа с ROS2 через Docker (Windows)

На Windows не требуется устанавливать ROS2 локально: весь стек запускается в контейнере. Нужны только Docker Desktop и репозиторий RTK2026.

## Требования

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) для Windows
- Клонированный репозиторий RTK2026

## Сборка образа

Из корня репозитория (папка, где лежат `src/`, `docker/`):

```powershell
cd c:\CursorProject\Robotics\RTK\RTK2026
docker compose -f docker/docker-compose.yml build
```

Либо:

```powershell
docker build -t rtk2026:latest -f docker/Dockerfile .
```

## Режимы запуска

### Интерактивная оболочка (разработка)

Запуск контейнера с bash; внутри доступны `ros2`, `colcon`, `pytest`:

```powershell
docker run --rm -it rtk2026:latest bash
```

Внутри контейнера:

```bash
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
ros2 launch rtk2026_bringup rtk2026_driver_base.launch.py use_description:=false use_fake_encoder:=true
# или любой другой launch
```

Второй терминал — ещё один контейнер с bash для публикации топиков:

```powershell
docker run --rm -it --network host rtk2026:latest bash
# на Windows host network может не работать — см. раздел про сеть ниже
```

### Запуск одного launch без входа в контейнер

Bringup по умолчанию:

```powershell
docker run --rm rtk2026:latest
```

Driver + base с fake encoder:

```powershell
docker run --rm rtk2026:latest bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_bringup rtk2026_driver_base.launch.py use_description:=false use_fake_encoder:=true"
```

Симуляция (Isaac Lab, use_sim_time + fake_scan + slam):

```powershell
docker run --rm -e ROS_DOMAIN_ID=0 rtk2026:latest bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_slam:=true"
```

Или используйте скрипт: `scripts\run_docker_simulation.ps1`.

### Gazebo + SLAM + Nav2 + explorer (RTK2026)

Новый сценарий симуляции, полностью основанный на самом проекте RTK2026 (без сторонних пакетов робота):

1. Убедитесь, что на хосте есть репозиторий миров Gazebo:

   - `C:\CursorProject\Robotics\gazebo_models_worlds_collection` (клонированный `leonhartyao/gazebo_models_worlds_collection`).

2. Из корня RTK2026:

   ```powershell
   cd C:\CursorProject\Robotics\RTK\RTK2026
   .\scripts\run_rtk2026_sim.ps1 -Build -Explore -World city
   ```

   - `-Build` — собрать образ `rtk2026:latest` из `docker/Dockerfile`;
   - `-Explore` — включить фронтир‑эксплорер (автоматическое исследование карты);
   - `-World`:
     - `city` → `/gazebo_worlds/worlds/small_city.world`;
     - `track` → `/gazebo_worlds/worlds/silverstone_track.world`;
     - любое другое значение трактуется как путь к `.world` **внутри контейнера**.

3. Внутри контейнера выполняется:

   ```bash
   ros2 launch rtk2026_bringup rtk2026_sim_slam_explore.launch.py world:=... x:=... y:=... z:=...
   ```

   Этот launch:

   - поднимает Gazebo с роботом RTK2026 (`rtk2026_simulation/rtk2026_gazebo.launch.py`);
   - стартует SLAM Toolbox + Nav2 с параметрами из `rtk2026_nav2_explorer/config/nav2_params_slam.yaml`;
   - запускает ноду `rtk2026_nav2_explorer/explorer`, которая находит фронтиры и отправляет цели в Nav2.

### Тесты

```powershell
docker run --rm rtk2026:latest bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 -m pytest /workspace/src/rtk2026_driver/test /workspace/src/rtk2026_base/test -v --tb=short"
```

## Сеть и Isaac Sim

Чтобы контейнер видел топики от Isaac Sim (или другого ROS2-узла на хосте), нужна общая сеть и один и тот же `ROS_DOMAIN_ID`. На Linux часто используют `network_mode: host`. На Windows в Docker Desktop по умолчанию контейнер и хост могут обмениваться по localhost, если симулятор публикует на том же интерфейсе; проверьте `ROS_DOMAIN_ID` (например, 0) и при необходимости настройте [ROS2 DDS через сеть](https://docs.ros.org/en/humble/How-To-Guides/DDS-setting.html).

### Ошибка Nav2: "Invalid frame ID odom … frame does not exist"

Она означает, что `controller_server`/local_costmap не видят фрейм `odom` в дереве TF. Частые причины:

- **Два контейнера (base + Nav2):** TF публикуется в одном контейнере, Nav2 — в другом; DDS между контейнерами на Windows часто не передаёт TF. Решение: использовать **один контейнер** с полным launch (base + Nav2 в одном процессе).
- **Один контейнер:** убедитесь, что запускаете с `use_fake_odom:=true` и `use_navigation:=true`. Тогда `static_odom_tf_publisher` публикует `map->odom` и `odom->base_link`, а узел `trigger_nav2_bringup` ждёт появления TF и только потом вызывает `manage_nodes` для Nav2. Скрипт: `scripts\run_docker_slam_one.ps1`.

Имена фреймов и таймауты задаются аргументами launch (`nav2_trigger_odom_frame`, `nav2_trigger_base_frame`, `nav2_trigger_delay_sec`, `nav2_trigger_tf_timeout_sec`) без хардкода в коде.

## Передача устройства (Arduino)

Чтобы отдать контейнеру последовательный порт Arduino на Windows, укажите устройство при запуске (подставьте свой COM-порт):

```powershell
docker run --rm -it --device COM3 rtk2026:latest bash
```

Внутри контейнера порт может отображаться как `/dev/ttyS3` или иначе; задайте параметр `serial_port` в конфиге драйвера или через launch.

## Скрипты

- `scripts/run_docker.ps1` — сборка (опционально) и интерактивный bash в контейнере
- `scripts/run_docker_simulation.ps1` — запуск simulation.launch в контейнере (для теста с Isaac Lab)
- `scripts/run_rtk2026_sim.ps1` — новый стек Gazebo + SLAM + Nav2 + explorer
- `scripts/run_rtk2026_diff_robot.ps1` — запуск вендорного diff_robot‑сетапа (как в ros2_diff_drive_robot)

### Isaac Sim + SLAM/Nav2

По образцу [zero-to-slam](https://github.com/Caian/zero-to-slam): Isaac Sim с ROS2-мостом публикует `/odom`, `/scan`; ROS2-стек в Docker — slam_toolbox, Nav2. Одом и tf идут из Isaac; odom_tf_broadcaster ретранслирует tf из `/odom`.

1. Запустите Isaac Sim с ROS2-мостом и сцену RTK2026, нажмите Play.
2. В отдельном терминале из корня RTK2026 выполните:

```powershell
cd C:\CursorProject\Robotics\RTK\RTK2026
.\scripts\run_docker_isaac_slam.ps1
```

В контейнере: robot_state_publisher, odom_tf_broadcaster, fake_scan, slam_toolbox, Nav2 (RViz в контейнере не запускается — нет дисплея; при необходимости RViz на хосте с тем же ROS_DOMAIN_ID). Подробно: [docs/isaac/ISAAC_ROS_SLAM.md](isaac/ISAAC_ROS_SLAM.md).

См. также [README.md](README.md) и [docs/isaac/ISAAC_SIM_TEST.md](isaac/ISAAC_SIM_TEST.md).
