# RTK2026

ROS2 workspace для мобильного робота RTK2026: база на колёсах, стереокамера, 2D-лидар, два мотора с энкодерами, IMU. Моторы управляются через Arduino Mega; Raspberry Pi 4 B запускает ROS2 и общается с Arduino и датчиками.

## Ссылки

- [RosTeamWS](https://github.com/b-robotized/ros_team_workspace) — структура workspace и пакетов
- [MentorPi](https://github.com/Hiwonder/MentorPi/tree/MentorPi-T1) — bringup, драйвер, SLAM, навигация

## Требования

- **Docker** — основной способ запуска на Windows; локальная установка ROS2 не обязательна. См. [DOCKER_WINDOWS.md](DOCKER_WINDOWS.md).
- Либо ROS2 Humble (Linux; при необходимости на Windows — [ROS2_WINDOWS_INSTALL.md](ROS2_WINDOWS_INSTALL.md)).
- Python 3.10+ (для локальных скриптов и тестов).

## Сборка (локально)

Из корня репозитория (workspace):

```bash
rosdep install -y -i --from-paths src
colcon build --symlink-install
source install/setup.bash
```

## Запуск через Docker

На Windows рекомендуется работать через Docker: сборка и запуск в контейнере, без установки ROS2. Подробно: [DOCKER_WINDOWS.md](DOCKER_WINDOWS.md). Кратко: `docker compose -f docker/docker-compose.yml build`, затем:

```bash
docker run --rm rtk2026:latest
```

По умолчанию запускается:

```bash
ros2 launch rtk2026_bringup rtk2026.launch.py
```

## Профили запуска

### Реальный робот (драйвер + база)

```bash
ros2 launch rtk2026_bringup rtk2026_driver_base.launch.py
```

Аргумент `publish_tf` (по умолчанию `true`): установите `false`, если используете EKF, чтобы TF `odom->base_link` публиковала только robot_localization.

**Полный стек** (драйвер, база, опционально локализация / SLAM / Nav2):

```bash
ros2 launch rtk2026_bringup full.launch.py use_description:=false use_fake_encoder:=true
ros2 launch rtk2026_bringup full.launch.py use_localization:=true   # EKF; база не публикует tf
ros2 launch rtk2026_bringup full.launch.py use_slam:=true          # slam_toolbox (нужен /scan)
ros2 launch rtk2026_bringup full.launch.py use_navigation:=true    # Nav2 (нужны карта и /scan)
```

Передайте последовательный порт в контейнер (например `--device /dev/ttyUSB0` на Linux или аналог для COM-порта Arduino на Windows). Параметр `serial_port` задаётся в `rtk2026_driver/config/arduino_bridge.yaml` или через launch.

При запуске без дисплея используйте `use_description:=false`, чтобы не поднимать RViz.

### Симуляция: RTK-робот (Gazebo + SLAM + Nav2 + explorer)

Скрипт `scripts/run_rtk2026_sim.ps1` — полная автономная симуляция на URDF RTK с `ros2_control`.

1. Клонируйте репозиторий миров Gazebo на хосте: `gazebo_models_worlds_collection` (репозиторий `leonhartyao/gazebo_models_worlds_collection`).
2. Из корня RTK2026:

   ```powershell
   .\scripts\run_rtk2026_sim.ps1 -Build -Explore -World city
   ```

   Параметры: `-Build` (пересборка образа), `-Explore` (фронтир-эксплорер), `-World` (`city` / `track` или путь к `.world` в контейнере).

В контейнере запускается `rtk2026_sim_slam_explore.launch.py`: Gazebo, RTK-URDF, SLAM, Nav2, explorer.

### Симуляция: diff_robot на трассе (тест RTK-навигации)

Скрипт `scripts/run_rtk2026_diff_robot.ps1` поднимает **пример симуляционной платформы** (пакет `diff_robot`) и поверх него — RTK-стек (SLAM, Nav2, explorer). Подробно: [RTK_DIFF_TRACK_SIM.md](RTK_DIFF_TRACK_SIM.md).

**Важно:** пакет `src/diff_robot` — это **не код RTK**. Это встроенный пример симуляционной платформы (URDF и миры из стиля `ros2_diff_drive_robot`), который используется **только для тестов навигации на трассе** (Silverstone и др.). Ядро проекта — пакеты `rtk2026_*`; `diff_robot` можно рассматривать как отдельный «профиль симуляции» для проверки SLAM/Nav2/explorer без реального железа.

Пример запуска:

```powershell
.\scripts\run_rtk2026_diff_robot.ps1 -World track
```

Управление с клавиатуры: в другом терминале `docker exec -it rtk2026_diff_robot_gazebo bash`, затем `source` окружение и `ros2 run teleop_twist_keyboard teleop_twist_keyboard` (образ должен быть собран с `ros-humble-teleop-twist-keyboard` в Dockerfile).

### Реальный робот + отладочный контейнер (RViz, логи, teleop)

Скрипт `scripts/run_rtk2026_robot_debug.ps1` — контейнер на ПК с SLAM, Nav2, explorer, RViz и `teleop_twist_keyboard`. Робот (Raspberry Pi) публикует `/odom`, `/scan`; контейнер строит карту и позволяет управлять роботом с клавиатуры. См. [RTK_DIFF_TRACK_SIM.md](RTK_DIFF_TRACK_SIM.md).

### Симуляция (Isaac Lab)

При запуске с Isaac Lab используйте `use_sim_time:=true`. Симулятор должен публиковать `/odom`, `/scan` и подписываться на `/cmd_vel`.

```bash
ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true
ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_slam:=true use_navigation:=true
```

Сцена Isaac Lab: каталог `isaac_lab/`, скрипт `run_rtk2026_scene.py`. Полный пошаговый тест в Isaac Sim: [isaac/ISAAC_SIM_TEST.md](isaac/ISAAC_SIM_TEST.md). На Windows запуск через Docker: `scripts/run_docker_simulation.ps1`.

## Тест без железа

Проверка пайплайна драйвер + база без Arduino и последовательного порта:

```bash
ros2 launch rtk2026_bringup rtk2026_driver_base.launch.py use_description:=false use_fake_encoder:=true
```

Запускаются `fake_encoder` (публикует `encoder_report` с нулевыми тиками) и `base_controller`. В другом терминале:

- `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"` — база будет публиковать `motor_command`.
- `ros2 topic echo /odom` — одометрия и TF `odom -> base_link`; поза остаётся нулевой, так как fake_encoder даёт нулевые приращения.

## Тесты

Из корня после сборки:

```bash
source install/setup.bash
python3 -m pytest src/rtk2026_driver/test src/rtk2026_base/test -v --tb=short
```

В Docker:

```bash
docker run --rm rtk2026:latest bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 -m pytest /workspace/src/rtk2026_driver/test /workspace/src/rtk2026_base/test -v --tb=short"
```

## Периферия (2D-лидар, камера, IMU)

Launch-файлы по структуре [MentorPi](https://github.com/Hiwonder/MentorPi/tree/MentorPi-T1). Идентификаторы фреймов совпадают с URDF: `lidar_link`, `camera_link`, `imu_link`.

- **2D-лидар** (топик `/scan`, фрейм `lidar_link`):
  - LD19: `ros2 launch rtk2026_peripherals lidar.launch.py` (нужен пакет `ldlidar_stl_ros2`)
  - MS200: `ros2 launch rtk2026_peripherals lidar_ms200.launch.py` (нужен пакет `oradar_lidar`)
  - Без железа: `ros2 launch rtk2026_peripherals fake_scan.launch.py`
- **USB-камера:** `ros2 launch rtk2026_peripherals depth_camera.launch.py` (нужен `usb_cam`). Для стерео/глубины — launch от RealSense или Orbbec.
- **Фильтр IMU** (сырой топик -> фильтрованный `/imu`, при необходимости TF imu_link -> imu_optical_frame): `ros2 launch rtk2026_peripherals imu_filter.launch.py` (нужен `imu_complementary_filter`). Задайте `imu_raw_topic` топиком вашего драйвера IMU. Для слияния IMU в EKF раскомментируйте `imu0` в `rtk2026_localization/config/ekf.yaml`.

## Документация

- [Docker на Windows](DOCKER_WINDOWS.md) — работа с ROS2 через контейнеры без локальной установки.
- [Протокол Arduino](protocol_arduino.md) — формат обмена 2 байта RX и 32 байта TX между RPi и Arduino.
- [Симуляция на трассе и отладка робота](RTK_DIFF_TRACK_SIM.md) — diff_robot на трассе, управление с клавиатуры, отладочный контейнер для реального робота.

## Структура репозитория

- `arduino/` — прошивка Arduino Mega (моторы, энкодеры).
- `docs/` — протоколы, инструкции, описание модулей (`docs/modules/*.md`).
- `docker/` — Dockerfile и docker-compose для сборки и запуска.
- `isaac_lab/` — скрипты сцен Isaac Lab (пол, свет; робот и ROS2-мост добавляются по документации).
- `src/` — пакеты ROS2:
  - **Ядро RTK:** `rtk2026`, `rtk2026_bringup`, `rtk2026_description`, `rtk2026_interfaces`, `rtk2026_driver`, `rtk2026_base`, `rtk2026_localization`, `rtk2026_slam`, `rtk2026_navigation`, `rtk2026_simulation`, `rtk2026_peripherals`, `rtk2026_nav2_explorer`.
  - **Пример симуляционной платформы (не код RTK):** `diff_robot` — URDF и миры для тестов навигации на трассе; используется только через launch `rtk2026_diff_robot_track.launch.py`.
