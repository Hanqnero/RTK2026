# Визуализация робота с ПК (Foxglove Studio)

Робот (Raspberry Pi) публикует ROS2-топики: одометрию, TF, скан лидара, карту. ПК (Mac или Windows) подключается через Foxglove Studio и отображает все данные в реальном времени.

## Архитектура

```
Raspberry Pi (10.40.69.246)            PC (Mac / Windows)
+---------------------------+          +---------------------+
| ROS2 ноды                 |          | Foxglove Studio     |
|  arduino_bridge           |   ws://  |  (нативное прил.)   |
|  base_controller          | -------> |  Карта, робот,      |
|  robot_state_publisher    |  :8765   |  одометрия, TF,     |
|  foxglove_bridge          |          |  скан лидара         |
+---------------------------+          +---------------------+
```

Pi и ПК должны быть в одной локальной сети.

## 1. На Raspberry Pi

`foxglove_bridge` уже включен в `docker/pi/Dockerfile` и запускается по умолчанию через CMD (`use_foxglove:=true`).

### Сборка и запуск

```bash
cd ~/RTK2026
docker build -t rtk2026:latest -f docker/pi/Dockerfile .
docker run -d --name rtk2026 --privileged -v /dev:/dev --network host rtk2026:latest
```

### Проверка

```bash
docker logs rtk2026 2>&1 | grep foxglove
```

Должно быть: `foxglove_bridge: WebSocket server started on port 8765`.

### Ручной запуск (если не через CMD)

```bash
docker exec -d rtk2026 bash -c "source /opt/ros/humble/setup.bash && \
  source /workspace/install/setup.bash && \
  ros2 launch rtk2026_bringup rtk2026_driver_base.launch.py use_rviz:=false use_foxglove:=true"
```

## 2. На ПК (Mac / Windows)

### Установка Foxglove Studio

Скачайте бесплатную версию: [https://foxglove.dev/download](https://foxglove.dev/download)

- macOS: `.dmg`
- Windows: `.exe` установщик

### Подключение

1. Откройте Foxglove Studio.
2. Выберите "Open connection".
3. Тип: **Foxglove WebSocket**.
4. URL: `ws://10.40.69.246:8765` (подставьте IP вашего Pi).
5. Нажмите "Open".

### Настройка панелей

Добавьте панели для визуализации:

- **3D** -- модель робота, TF-фреймы, скан лидара (`/scan`), карта (`/map`)
- **Map** -- 2D-карта из SLAM
- **Raw Messages** -- просмотр любых топиков (`/odom`, `/encoder_report`, `/motor_command`)
- **Diagnostics** -- состояние нод
- **Plot** -- графики значений (скорость, PWM)

### Отправка команд

Foxglove поддерживает публикацию сообщений. Через панель "Publish" можно отправлять `/cmd_vel` или `/motor_command` напрямую.

## Доступные топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/tf`, `/tf_static` | TF | Дерево фреймов робота |
| `/robot_description` | String | URDF модель |
| `/odom` | Odometry | Одометрия от base_controller |
| `/encoder_report` | EncoderReport | Данные энкодеров |
| `/motor_command` | MotorCommand | Команды моторов |
| `/scan` | LaserScan | 2D-лидар (когда подключен) |
| `/map` | OccupancyGrid | Карта от SLAM (когда запущен) |
| `/cmd_vel` | Twist | Команды скорости |

## Без Foxglove (альтернатива)

Если нужен полноценный RViz, используйте VNC-контейнер на Mac:

```bash
./scripts/run_rtk2026_diff_robot_mac_vnc.sh
```

Или Docker + X-сервер на Windows.
