# Foxglove Studio — подключение к роботу с Mac

Робот (Raspberry Pi) публикует ROS2-топики через `foxglove_bridge` на порту `8765`.
Mac подключается напрямую через Ethernet.

```
Raspberry Pi 192.168.2.2              Mac 192.168.2.1
┌──────────────────────────┐          ┌─────────────────────┐
│  ROS2 ноды               │          │  Foxglove Studio    │
│   sllidar_node  /scan    │  ws://   │  (Desktop-приложение│
│   slam_toolbox  /map     │ ──────►  │   ws://192.168.2.2  │
│   base_controller /odom  │  :8765   │   :8765)            │
│   usb_cam  /camera/...   │          └─────────────────────┘
│   foxglove_bridge        │
└──────────────────────────┘
```

---

## Подключение

1. Установить [Foxglove Studio Desktop](https://foxglove.dev/download) (не браузер — `ws://` блокируется HTTPS-страницами).
2. Open connection → **Foxglove WebSocket**.
3. URL: `ws://192.168.2.2:8765`
4. Open.

> Если Pi не видна напрямую (например, Docker VM на Mac изолирует сеть),
> используй SSH-туннель: `ssh -f -N -L 8765:localhost:8765 pi@192.168.2.2`
> и подключайся к `ws://localhost:8765`.

---

## Настройка панелей для 2D-карты

### 3D-панель (карта + скан + модель робота)

1. Добавить панель **3D**.
2. В левом дереве топиков включить:
   - `/map` — серая 2D-карта от SLAM
   - `/scan` — точки лидара
   - `/tf` + `/tf_static` — позиционирование
   - `/robot_description` — модель робота (коробка)
3. Камера: нажать **"Top"** (вид сверху) для 2D-режима.
   Или в настройках панели → Camera → установить `distance` большим, `phi = 0`.

### Убрать оси TF-фреймов

В настройках 3D-панели (шестерёнка) → раздел **Transforms** → `Axis scale: 0`
или отключить **"Show frame axes"**.

### Дополнительные панели

- **Image** → `/camera/image_raw` — картинка с камеры.
- **Raw Messages** → `/odom`, `/encoder_report` — числовые данные.
- **Plot** → `/odom/twist/twist/linear/x` — график скорости.
- **Publish** → `/cmd_vel` — отправка команд движения.

---

## Доступные топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/scan` | LaserScan | 2D-скан лидара (RPLIDAR C1) |
| `/map` | OccupancyGrid | Карта от SLAM Toolbox |
| `/odom` | Odometry | Одометрия |
| `/tf` | TFMessage | Динамические трансформы |
| `/tf_static` | TFMessage | Статические трансформы |
| `/robot_description` | String | URDF модель |
| `/camera/image_raw` | Image | USB-камера |
| `/cmd_vel` | Twist | Команды движения |
| `/encoder_report` | EncoderReport | Данные энкодеров |
| `/motor_command` | MotorCommand | Команды моторов |

---

## Проверка на Pi

```bash
ssh pi@192.168.2.2

# foxglove_bridge запущен?
docker logs rtk2026 2>&1 | grep foxglove

# Все ноды живы?
docker logs rtk2026 2>&1 | grep -E 'ERROR|died' | head -20
```
