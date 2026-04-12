# RTK2026

ROS2 Humble workspace для мобильного робота RTK2026.
Гусеничная база 300×250×113 мм, diff-drive, Raspberry Pi 5.

## Железо

| Компонент | Порт |
|-----------|------|
| Arduino (моторы + энкодеры) | `/dev/ttyUSB0` |
| LiDAR Slamtec RPLIDAR C1 | `/dev/ttyUSB2` |
| USB-камера | `/dev/video0` |
| Raspberry Pi 5 | `192.168.2.2` |
| Mac (через Ethernet) | `192.168.2.1` |

---

## Быстрый старт: Mac → Raspberry Pi

### 1. Сетевое подключение

Прямой Ethernet-кабель Mac ↔ Pi. На Mac включить **Internet Sharing**:
System Settings → General → Sharing → Internet Sharing → Share via Thunderbolt/USB Ethernet.

После этого Pi получает IP `192.168.2.2`, Mac — `192.168.2.1`.

```bash
ping 192.168.2.2      # проверить связь
ssh pi@192.168.2.2    # войти на Pi
```

Подробнее: [ETHERNET_MAC_PI_CONNECTION.md](ETHERNET_MAC_PI_CONNECTION.md)

### 2. Сборка Docker-образа на Mac (arm64)

```bash
./scripts/build_pi.sh
```

Или вручную:

```bash
docker buildx build --platform linux/arm64 -t rtk2026:latest -f docker/pi/Dockerfile .
```

Сборка ~15–20 мин (первый раз). Собирает: `sllidar_ros2`, `realsense-ros`, `foxglove_bridge` из исходников + все ROS2-пакеты.

### 3. Загрузка образа на Pi и запуск

```bash
./scripts/deploy_pi.sh
```

Или вручную:

```bash
# Передать образ (3–5 мин):
docker save rtk2026:latest | gzip | ssh pi@192.168.2.2 "gunzip | docker load"

# Запустить контейнер:
ssh pi@192.168.2.2 "docker run -d --name rtk2026 \
  --privileged -v /dev:/dev --network host rtk2026:latest"
```

### 4. Подключение Foxglove Studio

1. Открыть [Foxglove Studio](https://foxglove.dev/download) (Desktop-приложение).
2. Open connection → **Foxglove WebSocket** → `ws://192.168.2.2:8765`.
3. Добавить панель **3D** → добавить топики `/map`, `/scan`, `/tf`.

Подробнее: [FOXGLOVE_VIEWER.md](FOXGLOVE_VIEWER.md)

---

## Управление контейнером на Pi

```bash
ssh pi@192.168.2.2

# Статус
docker ps
docker logs rtk2026 --tail 30

# Перезапуск
docker restart rtk2026

# Остановка
docker stop rtk2026 && docker rm rtk2026

# Shell внутри контейнера
docker exec -it rtk2026 bash

# Освободить место (если диск заполнен)
docker system prune -af
```

---

## Топики

| Топик | Тип | Источник |
|-------|-----|----------|
| `/scan` | LaserScan | sllidar_node (RPLIDAR C1) |
| `/map` | OccupancyGrid | slam_toolbox |
| `/odom` | Odometry | base_controller |
| `/tf`, `/tf_static` | TF | robot_state_publisher, base_controller |
| `/robot_description` | String | robot_state_publisher |
| `/camera/image_raw` | Image | usb_cam |
| `/cmd_vel` | Twist | телеуправление |

---

## TF-дерево

```
map
 └── odom          ← slam_toolbox
      └── base_link  ← base_controller (odom→base_link)
           ├── lidar_link   ← robot_state_publisher (static)
           ├── camera_link  ← robot_state_publisher (static)
           └── imu_link     ← robot_state_publisher (static)
```

> **Важно:** `odom→base_link` публикуется только при наличии данных с энкодеров.
> Без подключённых моторов SLAM не может трансформировать скан лидара.
> Временный обход: добавить `static_transform_publisher odom base_link`.

---

## Параметры запуска (`pi_full.launch.py`)

```bash
# Без SLAM (только база + сенсоры + foxglove):
ros2 launch rtk2026_bringup pi_full.launch.py use_slam:=false

# Без камеры:
ros2 launch rtk2026_bringup pi_full.launch.py use_camera:=false

# Intel RealSense вместо usb_cam:
ros2 launch rtk2026_bringup pi_full.launch.py camera_driver:=realsense
```

---

## Симуляция (Gazebo, только для разработки на Mac/Windows)

| Платформа | Скрипт |
|-----------|--------|
| macOS (VNC) | `./scripts/run_rtk2026_diff_robot_mac_vnc.sh` |
| Windows (PowerShell) | `./scripts/run_rtk2026_diff_robot.ps1` |

Подробнее: [RTK_DIFF_TRACK_SIM.md](RTK_DIFF_TRACK_SIM.md), [RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md](RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md)

---

## Документация

- [Ethernet: Mac ↔ Pi](ETHERNET_MAC_PI_CONNECTION.md)
- [Foxglove Studio](FOXGLOVE_VIEWER.md)
- [Калибровка RealSense](CALIBRATION.md)
- [Протокол Arduino](protocol_arduino.md)
- [Спецификация моторов](MOTOR_SPEC.md)
- [Энкодер MT6701](ENCODER_MT6701.md)
- [Docker на Windows](DOCKER_WINDOWS.md)

---

## Структура репозитория

```
arduino/          — прошивка Arduino (моторы, энкодеры)
docker/pi/        — Dockerfile для Raspberry Pi (arm64)
docker/windows/   — Dockerfile для симуляции на Windows
scripts/
  build_pi.sh     — сборка образа на Mac
  deploy_pi.sh    — загрузка образа на Pi + запуск
src/
  rtk2026_interfaces/   — кастомные msg (EncoderReport, MotorCommand)
  rtk2026_description/  — URDF xacro
  rtk2026_driver/       — arduino_bridge (serial ↔ ROS2)
  rtk2026_base/         — base_controller (одометрия, TF, cmd_vel)
  rtk2026_bringup/      — launch-файлы (pi_full.launch.py)
  rtk2026_peripherals/  — lidar.launch.py, depth_camera.launch.py
  rtk2026_slam/         — slam_toolbox конфиг
  rtk2026_navigation/   — Nav2 конфиг
  rtk2026_localization/ — robot_localization (EKF)
  diff_robot/           — URDF для симуляции (не RTK)
```
