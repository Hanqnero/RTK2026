# Калибровка камеры RealSense

Intel RealSense D435i/D455 поставляется с заводской калибровкой встроенной в прошивку.
Параметры камеры публикуются автоматически в `/camera/color/camera_info`.

---

## Foxglove layouts

Импортируй в Foxglove Studio: **Layouts → Import from file...**

| Файл | Назначение |
|------|-----------|
| `foxglove_layouts/calibration_intrinsic.json` | Просмотр цвета + глубины + camera_info + IMU |
| `foxglove_layouts/calibration_extrinsic.json` | 3D-панель: лидар + камера + TF для проверки позиции |

---

## 1. Внутренняя (intrinsic) калибровка

### Заводская калибровка (по умолчанию)

RealSense публикует матрицу камеры сама по себе. Открой layout
`calibration_intrinsic.json` в Foxglove и проверь `/camera/color/camera_info`:

```
fx, fy — фокусные расстояния (пиксели)
cx, cy — главная точка (центр изображения)
D      — коэффициенты дисторсии (должны быть малыми для D435/D455)
```

### Переколибровка (если заводская не подходит)

Нужна шахматная доска 8×6 клеток, размер клетки ~25 мм.

**1. На Pi (через SSH с X11):**

```bash
ssh -X pi@192.168.2.2
docker exec -it rtk2026 bash

# Запустить калибровщик:
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/camera/color/image_raw \
  camera:=/camera/color
```

**2. Двигать шахматную доску** по всем углам и расстояниям, пока статус-бары не заполнятся:
- X / Y / Size / Skew → зелёные

**3. Нажать CALIBRATE → SAVE** → сохраняется в `/tmp/calibrationdata.tar.gz`.

**4. Скопировать на Mac и распаковать:**

```bash
docker cp rtk2026:/tmp/calibrationdata.tar.gz .
tar -xzf calibrationdata.tar.gz
# внутри: ost.yaml с матрицей камеры
```

**5. Опционально:** передать файл калибровки в realsense_camera.launch через
`color_camera.color_info_url:=file:///path/to/ost.yaml`.

---

## 2. Внешняя (extrinsic) калибровка

Цель: убедиться что `camera_link` в URDF совпадает с реальным физическим положением камеры.

### Шаг 1 — Измерить физическое положение камеры

Измерить расстояния от центра робота (base_link):

| Параметр | Описание | Текущее значение (URDF) |
|----------|----------|------------------------|
| x | вперёд от центра base_link (м) | `0.12` |
| y | вбок (0 = по центру) | `0` |
| z | высота от нижней грани base_link | `0.06` |

Файл: [src/rtk2026_description/urdf/rtk2026_macro.xacro](../src/rtk2026_description/urdf/rtk2026_macro.xacro)

```xml
<joint name="${prefix}camera_joint" type="fixed">
  <origin xyz="0.12 0 0.06" rpy="0 0 0"/>
</joint>
```

Отредактировать `xyz` по реальным замерам.

### Шаг 2 — Проверить в Foxglove

Открыть layout `calibration_extrinsic.json`:
- 3D панель → вид сверху
- Должны совпадать: красная точка лидара на стене = край, который видит камера
- `camera_link` frame должен находиться в правильной позиции относительно `base_link`

### Шаг 3 — Пересобрать после изменения URDF

После изменения `rtk2026_macro.xacro` нужно пересобрать образ:

```bash
./scripts/build_pi.sh
./scripts/deploy_pi.sh
```

Или внутри контейнера (без пересборки образа):

```bash
ssh pi@192.168.2.2
docker exec -it rtk2026 bash
cd /workspace
colcon build --symlink-install --packages-select rtk2026_description
# restart robot_state_publisher:
# kill -HUP $(pgrep -f robot_state_publisher)
```

---

## 3. IMU калибровка (RealSense)

RealSense D435i/D455 имеет встроенный IMU (акселерометр + гироскоп).

### Топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/camera/imu` | sensor_msgs/Imu | Объединённый IMU (accel + gyro, unite_imu_method=2) |
| `/camera/gyro/sample` | sensor_msgs/Imu | Только гироскоп (400 Hz) |
| `/camera/accel/sample` | sensor_msgs/Imu | Только акселерометр (250 Hz) |

### Использование в EKF

EKF (`robot_localization`) настроен на `use_localization:=true`:

```bash
ros2 launch rtk2026_bringup pi_full.launch.py use_localization:=true
```

Конфиг: [src/rtk2026_localization/config/ekf.yaml](../src/rtk2026_localization/config/ekf.yaml)
- `imu0: /camera/imu`
- Используется: yaw rate (`angular_velocity.z`) + linear accel x/y
- Ориентация не используется (RealSense не выдаёт фьюзированную ориентацию)

### Калибровка смещения IMU (при необходимости)

Если `/camera/imu` показывает drift в покое:

```bash
# Оставить робота неподвижным на 30 сек, смотреть на значения:
ssh pi@192.168.2.2
docker exec -it rtk2026 bash -c \
  "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   ros2 topic echo /camera/imu --once"
# linear_acceleration.z должно быть ~9.8, angular_velocity ~0
```

---

## TF-дерево с RealSense

```
map
 └── odom          ← EKF (при use_localization:=true)
      └── base_link
           ├── lidar_link          ← robot_state_publisher (static)
           ├── camera_link         ← robot_state_publisher (static, из URDF)
           │    ├── camera_color_frame
           │    │    └── camera_color_optical_frame
           │    ├── camera_depth_frame
           │    └── camera_imu_frame  ← realsense2_camera (static)
           └── imu_link
```
