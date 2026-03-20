# Пакет `rtk2026_peripherals` (лидар, камера, IMU и вспомогательные узлы)

## Назначение

Сценарии запуска и узлы для периферийных датчиков и вспомогательных сервисов:

- 2D‑лидар (`/scan`);
- камера;
- IMU‑фильтр;
- вспомогательные издатели TF/clock/map для симуляционных режимов (`fake_*`).

## Основные launch‑файлы

- Лидар:
  - `lidar.launch.py` (например, LD19, требует внешний пакет драйвера);
  - `lidar_ms200.launch.py` (MS200, также внешний пакет);
  - `fake_scan.launch.py` — публикация искусственного `/scan` для тестов/симуляции.
- Камера:
  - `depth_camera.launch.py` — USB‑камера (использует `usb_cam` или аналогичный драйвер).
- IMU:
  - `imu_filter.launch.py` — фильтрация IMU и, при необходимости, TF `imu_link` → `imu_optical_frame`.
- Вспомогательные узлы для Docker/симуляции:
  - `odom_tf_broadcaster.launch.py` — TF из `/odom`;
  - узлы `static_odom_tf_publisher`, `clock_publisher`, `static_map_publisher` и `trigger_nav2_bringup` для сборок, где Nav2 поднимают отдельным контейнером или с задержкой.

## Основные топики

- `/scan` (`sensor_msgs/LaserScan`) — лидара или `fake_scan`;
- `/camera/image_raw`, `/camera/camera_info` — камеры;
- `/imu/data` и производные (`/imu`) — IMU‑фильтр;
- служебные топики TF и `/clock` для симуляции.

