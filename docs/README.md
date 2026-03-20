# RTK2026

ROS2 workspace для мобильного робота RTK2026: гусеничная база, стереокамера, 2D-лидар, два мотора с энкодерами, IMU. Моторы управляются через Arduino Mega; Raspberry Pi 5 запускает ROS2 и общается с Arduino и датчиками.

## Платформы и Dockerfile

| Платформа | Dockerfile | Назначение |
|-----------|-----------|------------|
| **Raspberry Pi** (arm64) | `docker/pi/Dockerfile` | Headless: драйвер, база, SLAM, Nav2. Без Gazebo и RViz. |
| **Windows** (amd64) | `docker/windows/Dockerfile` | Gazebo + diff_robot + SLAM + Nav2 + explorer. Полная симуляция на трассе. |
| **macOS M1/M2/M3** | `ros-humble-desktop-m1_2-mac/Dockerfile` | Desktop-образ с VNC. Gazebo headless, RViz через TigerVNC. |

## Быстрый старт

### Raspberry Pi

```bash
cd ~/RTK2026
docker build -t rtk2026:latest -f docker/pi/Dockerfile .
docker run -d --name rtk2026 --privileged -v /dev:/dev --network host rtk2026:latest
```

По умолчанию запускается `rtk2026_driver_base.launch.py use_rviz:=false` (arduino_bridge + base_controller + robot_state_publisher).

### Windows (PowerShell)

```powershell
cd C:\path\to\RTK2026
.\scripts\run_rtk2026_diff_robot.ps1 -Build -Explore -World track
```

Подробно: [DOCKER_WINDOWS.md](DOCKER_WINDOWS.md), [RTK_DIFF_TRACK_SIM.md](RTK_DIFF_TRACK_SIM.md).

### macOS (M1/M2/M3)

```bash
cd /path/to/RTK2026
chmod +x scripts/run_rtk2026_diff_robot_mac_vnc.sh
./scripts/run_rtk2026_diff_robot_mac_vnc.sh
```

Подключение: TigerVNC Viewer -> `localhost:5900`. Подробно: [RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md](RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md).

## Сборка (локально)

```bash
rosdep install -y -i --from-paths src
colcon build --symlink-install
source install/setup.bash
```

## Профили запуска

### Реальный робот (драйвер + база)

```bash
ros2 launch rtk2026_bringup rtk2026_driver_base.launch.py
```

Аргументы:
- `use_rviz` (default `true`): `false` для headless (Pi).
- `publish_tf` (default `true`): `false` если используется EKF.
- `use_fake_encoder` (default `false`): `true` для теста без Arduino.

### Полный стек

```bash
ros2 launch rtk2026_bringup full.launch.py use_fake_encoder:=true
ros2 launch rtk2026_bringup full.launch.py use_localization:=true
ros2 launch rtk2026_bringup full.launch.py use_slam:=true
ros2 launch rtk2026_bringup full.launch.py use_navigation:=true
```

### Симуляция: diff_robot на трассе

Скрипт `scripts/run_rtk2026_diff_robot.ps1` (Windows) или `scripts/run_rtk2026_diff_robot_mac_vnc.sh` (Mac). Запускает Gazebo с diff_robot на трассе Silverstone + SLAM + Nav2 + explorer.

Подробно: [RTK_DIFF_TRACK_SIM.md](RTK_DIFF_TRACK_SIM.md).

**Пакет `src/diff_robot`** -- пример симуляционной платформы (не код RTK). Используется только для тестов навигации на трассе.

## Тест без железа

```bash
ros2 launch rtk2026_bringup rtk2026_driver_base.launch.py use_description:=false use_fake_encoder:=true
```

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
ros2 topic echo /odom
```

## Тесты

```bash
source install/setup.bash
python3 -m pytest src/rtk2026_driver/test src/rtk2026_base/test -v --tb=short
```

## Периферия (2D-лидар, камера, IMU)

- **2D-лидар** (`/scan`, `lidar_link`): `ros2 launch rtk2026_peripherals lidar.launch.py`
- **USB-камера:** `ros2 launch rtk2026_peripherals depth_camera.launch.py`
- **Фильтр IMU:** `ros2 launch rtk2026_peripherals imu_filter.launch.py`

## Документация

- [Визуализация робота с ПК (Foxglove)](FOXGLOVE_VIEWER.md) -- подключение Mac/Windows к Pi
- [Docker на Windows](DOCKER_WINDOWS.md)
- [Симуляция на трассе (Windows)](RTK_DIFF_TRACK_SIM.md)
- [Симуляция на Mac (VNC)](RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md)
- [Протокол Arduino](protocol_arduino.md) -- 4 байта RX, 16 байт TX
- [Спецификация моторов](MOTOR_SPEC.md)
- [Энкодер MT6701](ENCODER_MT6701.md)

## Структура репозитория

- `arduino/` -- прошивка Arduino Mega (моторы, энкодеры).
- `docs/` -- протоколы, инструкции.
- `docker/` -- `pi/Dockerfile` (Raspberry Pi), `windows/Dockerfile` (Windows).
- `ros-humble-desktop-m1_2-mac/` -- базовый desktop-образ для Mac (VNC, Xvfb).
- `scripts/` -- скрипты запуска для Windows (.ps1) и Mac (.sh).
- `src/` -- пакеты ROS2:
  - **Ядро RTK:** `rtk2026`, `rtk2026_bringup`, `rtk2026_description`, `rtk2026_interfaces`, `rtk2026_driver`, `rtk2026_base`, `rtk2026_localization`, `rtk2026_slam`, `rtk2026_navigation`, `rtk2026_simulation`, `rtk2026_peripherals`, `rtk2026_nav2_explorer`.
  - **Симуляция:** `diff_robot` -- URDF и миры для тестов навигации на трассе.
