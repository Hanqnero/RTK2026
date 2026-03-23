# Автономное вождение: полосы и знаки (TurtleBot3 Autorace)

Задача: езда без пересечения полосы и детекция дорожных знаков с принятием решений. В проекте используется мир и методика [TurtleBot3 Autonomous Driving](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/) (ROBOTIS e-Manual): симуляция с полосами, светофорами и знаками.

**Справка по официальному AutoRace:** симулятор — Gazebo; launch — `turtlebot3_autorace_2020.launch` (пакет turtlebot3_gazebo); основной world-файл трассы — **`turtlebot3_autorace_2020.world`** (в репозитории turtlebot3_simulations только этот мир; вариант `turtlebot3_autorace.world` в текущей версии отсутствует). Мы используем тот же world-файл и тот же набор моделей (GAZEBO_MODEL_PATH); в Gazebo спаунится наш **diff_robot** (не TurtleBot3), поверх — RTK SLAM и Nav2.

**Важно:** карта с дорогами и знаками получается только при запуске мира **autorace** (`turtlebot3_autorace_2020.world`). При `-World track` или `-World city` загружается другой мир (трасса Silverstone или город без разметки) — в нём нет полос и знаков; SLAM строит карту того мира, который реально загружен в Gazebo.

## Мир Autorace вместо трассы

Мир **TurtleBot3 Autorace 2020** (полоса, знаки, светофоры) можно поднять вместо Silverstone трассы с тем же diff_robot и RTK-стеком (SLAM, Nav2, teleop). Для autorace скрипт: передаёт в контейнер **GAZEBO_MODEL_PATH** (только каталог с turtlebot3_autorace_2020), чтобы Gazebo находил course и знаки; спаунит робота в точке старта как в официальном launch (`x:=0.8 y:=-1.747 z:=0.05`); использует уменьшенную модель **diff_robot_fifth.urdf** (масштаб 0.1: по высоте не выше зданий, по ширине — в пределах дороги), чтобы робот не задевал стены. Задержки: `spawn_delay:=50`, `nav2_delay:=70`. При необходимости координаты можно переопределить через `-SpawnX`, `-SpawnY`, `-SpawnZ`.

### 1. Клонирование turtlebot3_simulations

На хосте (рядом с RTK2026):

```powershell
cd C:\CursorProject\Robotics
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Для ROS2 Humble при необходимости переключитесь на ветку с поддержкой ROS2 (например `humble` или `ros2`, если есть в репозитории).

### 2. Запуск симуляции в мире Autorace

Из корня RTK2026:

```powershell
.\scripts\run_rtk2026_diff_robot.ps1 -World autorace
```

С управлением с клавиатуры и с использованием видеокарты (если в Docker доступна GPU):

```powershell
.\scripts\run_rtk2026_diff_robot.ps1 -World autorace -Teleop -UseGpu
```

**Пересборка образа:** уменьшенный робот (diff_robot_fifth.urdf) лежит в `src/diff_robot/urdf/`. В контейнер попадает только то, что было в образе при сборке. Чтобы размер робота в autorace изменился, пересоберите образ: `.\scripts\run_rtk2026_diff_robot.ps1 -Build -World autorace` (один раз с `-Build`, дальше можно без него).

Если на хосте есть клон `C:\CursorProject\Robotics\turtlebot3_simulations`, скрипт монтирует его и задаёт **GAZEBO_MODEL_PATH** равным только этому каталогу моделей (`.../turtlebot3_gazebo/models`) — тогда Gazebo подгружает course и знаки. Убедитесь, что в клоне есть папка `turtlebot3_gazebo/models/turtlebot3_autorace_2020/` (course, traffic_light и др.). Без клона мир берётся из образа (нужен `-Build`). В autorace спаунится уменьшенная модель **diff_robot_fifth.urdf** (масштаб 0.1, по габаритам под дорогу и здания); топики те же: камера, лидар, /cmd_vel, /odom, /scan.

**GPU в Docker (Windows):** чтобы Gazebo использовал видеокарту, добавьте `-UseGpu`. Нужны: Docker Desktop на WSL2, в WSL2 установлен [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (nvidia-docker2). Тогда контейнер запускается с `--gpus all` и OpenGL/Gazebo могут использовать GPU. Без этого рендер идёт через программный вывод (медленнее).

## Lane detection и детекция знаков (TurtleBot3 Autorace)

Логика «езда по полосе» и «детекция знаков + решения» реализована в пакетах ROBOTIS:

- **turtlebot3_autorace** — мета-пакет.
- **turtlebot3_autorace_camera** — калибровка камеры (intrinsic/extrinsic).
- **turtlebot3_autorace_detect** — детекция полосы (lane), светофоров, знаков (traffic sign), переезда (level crossing) и т.д.
- **turtlebot3_autorace_mission** — миссии: traffic_light, intersection, construction, parking, level_crossing, tunnel.

Официальная документация и порядок запуска: [Autonomous Driving — e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/). Ниже — краткая схема интеграции с нашей симуляцией.

### Установка пакетов Autorace (в контейнере или отдельном workspace)

Для ROS2 Humble:

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git
cd ~/turtlebot3_ws && colcon build --symlink-install
```

Дополнительные зависимости (на машине с ROS2):

```bash
sudo apt install ros-humble-image-transport ros-humble-cv-bridge ros-humble-vision-opencv python3-opencv libopencv-dev ros-humble-image-pipeline
```

В Docker-образ RTK2026 эти пакеты по умолчанию не добавлены; при необходимости их можно включить в Dockerfile и собрать отдельный образ или поднимать autorace-узлы в отдельном контейнере с общим `ROS_DOMAIN_ID` и сетью.

### Согласование топиков с diff_robot

Наш diff_robot публикует:

- `/camera/image_raw`, `/camera/camera_info`
- `/scan`, `/odom`, `/cmd_vel`

Узлы turtlebot3_autorace_camera и turtlebot3_autorace_detect обычно ожидают топики камеры вроде `/camera/image_raw` или `/camera/image_rect`. При необходимости задайте remap при запуске их launch-файлов, чтобы они читали наши топики.

Пример (идея): запуск детекции полосы с указанием топика камеры:

```bash
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py  # при необходимости добавьте remap в launch или параметры
```

Детекция знаков (миссия выбирается аргументом `mission`):

```bash
ros2 launch turtlebot3_autorace_detect detect_sign.launch.py mission:=intersection
```

Миссии (traffic_light, intersection, construction, parking, level_crossing, tunnel) описаны в e-Manual в разделе [Missions](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/).

### Рекомендуемый порядок (по e-Manual)

1. Запустить симуляцию в мире Autorace (у нас: `-World autorace` + при необходимости `-Teleop`).
2. Запустить калибровку камеры (intrinsic, затем extrinsic), если используете узлы autorace.
3. Запустить детекцию полосы (detect_lane) и при необходимости управление по полосе (control_lane).
4. Запустить детекцию знаков (detect_sign с нужной mission) и миссионные узлы (turtlebot3_autorace_mission).

Подробные шаги, калибровка и настройка параметров — в [Camera Calibration](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#camera-calibration), [Lane Detection](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#lane-detection), [Traffic Sign Detection](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#traffic-sign-detection) и [Missions](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#missions) e-Manual.

### Официальная процедура обнаружения полос (e-Manual)

По e-Manual обнаружение полос делается так:

1. **Симуляция** — запуск Gazebo с миром Autorace:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
   ```
2. **Калибровка камеры** — внутренняя и внешняя, чтобы получить корректное сопоставление полос с перспективой робота:
   ```bash
   ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
   ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
   ```
   Эти шаги формируют вид «сверху» (bird's eye) в топиках `/camera/image_projected` и т.п.
3. **Детекция полос в режиме калибровки** — для настройки порогов по цвету:
   ```bash
   ros2 launch turtlebot3_autorace_detect detect_lane.launch.py calibration_mode:=True
   ```
4. **Проверка в rqt** — Plugins > Visualization > Image View, топики:
   - `/detect/image_lane/compressed` — результат обнаружения полосы;
   - `/detect/image_yellow_lane_marker/compressed` — маска по жёлтому;
   - `/detect/image_white_lane_marker/compressed` — маска по белому.
5. **Калибровка параметров полосы** — правка `lane.yaml` в `turtlebot3_autorace_detect/param/lane/` (диапазоны hue/saturation/lightness для белой и жёлтой разметки). Сохранённые значения используются при следующих запусках.
6. **Рабочий запуск** — без режима калибровки:
   ```bash
   ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
   ros2 launch turtlebot3_autorace_mission control_lane.launch.py
   ```

В пайплайне RTK (см. ниже) вместо калибровки turtlebot3_autorace_camera по умолчанию используется узел **image_relay_autorace** с фиксированным IPM под diff_robot_fifth. Для стабильной езды по полосе (без ухода в круг и без частого Lane state: 0) нужно один раз откалибровать камеру и настроить пороги детекции полосы.

### Калибровка в пайплайне RTK (режимы скрипта)

Скрипт поддерживает два режима калибровки (только с `-Autorace`):

1. **Режим калибровки полосы** (`-LaneCalibration`) — запускает detect_lane с `is_detection_calibration_mode:=true`. В rqt (Plugins > Visualization > Image View) смотрите топики `/detect/image_lane/compressed`, `/detect/image_yellow_lane_marker/compressed`, `/detect/image_white_lane_marker/compressed` и через Dynamic Reconfigure подстраиваете пороги (hue/saturation/lightness для жёлтой и белой разметки). Сохранённые значения записываются в `lane.yaml` пакета turtlebot3_autorace_detect (см. ниже про сохранение между запусками).

2. **Калибровка камеры** (`-UseCameraCalibration`) — поднимает узлы **turtlebot3_autorace_camera** (intrinsic и extrinsic); вид «сверху» для полосы берётся из них (`/camera/image_projected`) и показывается в RViz. При этом наш IPM-релей продолжает работать (но публикует в отдельный топик `/rtk_autorace_ipm/...`), поэтому робот не “встаёт”, даже если по какой-то причине калибровочные топики появились не сразу.

**Рекомендуемый порядок один раз:**

1. Запуск с калибровкой камеры и полосы:
   ```powershell
   .\scripts\run_rtk2026_diff_robot.ps1 -World autorace -Autorace -UseCameraCalibration -LaneCalibration
   ```
   По умолчанию `detect_lane` продолжает использовать IPM-картинку (`lane_image_source:=ipm`), чтобы робот не зависал, даже если калибровочные топики появились не сразу. Если хотите, чтобы `detect_lane` питался именно от `/camera/image_projected`, добавьте `-LaneFromCamera`.
2. После старта пайплайна: в rqt подстроить маски полосы (см. выше), при необходимости подправить extrinsic в топиках `/camera/image_extrinsic_calib` и `/camera/image_projected`.
3. Сохранить параметры полосы в `lane.yaml` (в контейнере путь: `install/turtlebot3_autorace_detect/share/turtlebot3_autorace_detect/param/lane/lane.yaml`). Чтобы они не терялись при следующем запуске — скопировать файл с контейнера на хост и при следующей сборке образа положить его в образ, либо смонтировать том (см. ниже).
4. Дальше запускать без калибровки (и при желании без камеры калибровки, с IPM):
   ```powershell
   .\scripts\run_rtk2026_diff_robot.ps1 -World autorace -Autorace
   ```

**Сохранение lane.yaml между запусками:** параметры хранятся в образе в `install/.../param/lane/lane.yaml`. После настройки в режиме калибровки скопируйте файл из контейнера:
   ```powershell
   docker cp rtk2026_diff_robot_gazebo:/workspace/install/turtlebot3_autorace_detect/share/turtlebot3_autorace_detect/param/lane/lane.yaml .\docker\param\lane.yaml
   ```
   Затем в Dockerfile можно добавить копирование этого файла в образ (перезапись дефолтного lane.yaml) при сборке, либо смонтировать каталог с хоста в контейнер (например `-v C:\path\to\param\lane:/workspace/install/turtlebot3_autorace_detect/share/turtlebot3_autorace_detect/param/lane`) при запуске скрипта (потребуется доработка скрипта под volume для lane).

## Интеграция в пайплайн RTK (lane + sign + obstacle)

Функционал TurtleBot3 Autorace интегрирован в RTK:

- **Езда по полосе** — detect_lane + control_lane (публикуют в `/cmd_vel_raw`). **Объезд** — avoid_construction (публикует `/avoid_control` и `/avoid_active`). Узел **cmd_vel_limiter** при `use_avoid_merge:=true` при активном объезде подаёт в `/cmd_vel` команды из `/avoid_control`, иначе — ограниченный `/cmd_vel_raw`.
- **Детекция знаков** — detect_construction_sign (и при необходимости другие mission).
- **Объезд препятствий** — avoid_construction (лидар `/scan` + состояние полосы + одометрия). Зона опасности и габариты робота задаются параметрами под **diff_robot_fifth** (0.08 x 0.05 м), чтобы не реагировать на объекты «за линией» (вне полосы) и вовремя тормозить перед препятствиями по курсу.

**Связка с картой (SLAM):** при запуске Autorace **по умолчанию** стартует **slam_toolbox** (конфиг `slam_toolbox_autorace.yaml`): карта строится по `/scan` и `/odom` и отображается в RViz. **Робот эту карту не использует для управления** — движение задаётся только полосой/знаками/объездом (не Nav2). Чтобы не строить карту, передайте в скрипте **-NoMap**.

Запуск симуляции в мире Autorace с полным пайплайном (полоса, знаки, объезд; без Nav2):

```powershell
.\scripts\run_rtk2026_diff_robot.ps1 -World autorace -Autorace
```

Карта по умолчанию строится (slam_toolbox + RViz с отображением карты). Угловая и линейная скорость ограничены, чтобы робот не крутился на месте. Чтобы не запускать SLAM, передайте **-NoMap**.

Для калибровки камеры и порогов полосы (один раз) используйте **-UseCameraCalibration** и **-LaneCalibration** (см. раздел «Калибровка в пайплайне RTK» выше).

При первом запуске с новым образом пересоберите образ: `-Build -World autorace -Autorace`. После изменения патчей в `docker/patches/` (в т.ч. параметризация avoid_construction) образ нужно пересобрать.

Внутри контейнера: релей **image_relay_autorace** переводит `/camera/image_raw` в вид «сверху» (IPM) и публикует в `/camera/image_projected` и `/camera/image_compensated`, затем запускаются узлы turtlebot3_autorace_detect и turtlebot3_autorace_mission. Ограничения объезда препятствий см. ниже.

### Детекция полосы: одна камера и bird's eye (IPM)

Используется **одна камера переднего вида**. Алгоритм detect_lane рассчитан на **вид сверху** на дорогу (bird's eye / IPM). В нашем пайплайне релей **image_relay_autorace** после ресайза до 1000x600 применяет **IPM** по параметрам камеры (высота над землёй, внутренняя калибровка) и отдаёт в detect_lane уже спроектированный кадр, так что дополнительная калибровка turtlebot3_autorace_camera не нужна.

**Параметры IPM** (узла image_relay_autorace, при необходимости задаются в launch или через параметры):

- `use_ipm` (bool) — включить преобразование в bird's eye (по умолчанию true).
- `camera_height_m` — высота оптического центра камеры над плоскостью пола (для diff_robot_fifth: 0.025).
- `y_near_m`, `y_far_m` — ближняя и дальняя граница участка дороги по курсу (м), например 0.05 и 2.0.
- `x_left_m`, `x_right_m` — ширина участка слева/справа (м), например -0.4 и 0.4.
- `fx`, `fy`, `cx`, `cy` — внутренняя калибровка камеры для изображения 1000x600 (по умолчанию под Gazebo-камеру diff_robot: 640x480, horizontal_fov 1.396, после ресайза).

Для другого робота или реальной камеры подставьте свои значения (высота из URDF, при необходимости калибровка из `/camera/camera_info`). Если робот пересекает линии: проверьте в RViz или rqt_image_view вид `/camera/image_projected` (должны быть две чёткие полосы по краям); при необходимости увеличьте `x_left_m`/`x_right_m` (ширина кадра в метрах) или уменьшите `y_far_m`. Узел **autorace_max_vel_publisher** публикует `/control/max_vel` (по умолчанию 0.032), чтобы снизить скорость и стабилизировать езду по полосе; при необходимости измените параметр `max_vel` в launch.

### Почему робот может врезаться в светофор, даже если на карте препятствие есть

Модуль объезда препятствий **avoid_construction** использует только **лидар** (`/scan`): он строит зону опасности впереди робота (параметры danger_distance, danger_width) и по точкам лидара решает, когда включать объезд. Карта в RViz строится тем же лидаром (slam_toolbox), поэтому препятствие на карте и в avoid_construction — один и тот же источник данных. Но:

- **Светофор** часто смоделирован как столб (тонкий объект). 2D-лидар даёт один срез по высоте; столб может попасть между лучами или дать мало точек. В пайплайне зона опасности увеличена (danger_distance 0.22 м, danger_width 0.14 м), чтобы раньше засекать препятствия.
- Зона опасности — прямоугольник **впереди** робота по курсу; при выезде с полосы робот может подъезжать к светофору под углом. Убедитесь, что в `/cmd_vel` при объезде идут команды от avoid_construction (в launch включён **use_avoid_merge** у cmd_vel_limiter).
- То, что препятствие «отобразилось на карте», значит только, что лидар его когда-то увидел; логика объезда срабатывает по текущему скану и ограниченной зоне, а не по готовой карте.

Итого: сбой по полосе (из-за отсутствия IPM) приводит к выезду с трассы; объезд по лидару не рассчитан на тонкие вертикальные объекты (светофорный столб) и может их пропустить. Для более надёжного избегания таких препятствий нужна либо доработка зоны/логики avoid_construction, либо использование 3D-данных (например, глубина с камеры), если они есть на роботе.

## Итог

| Что нужно | Действие |
|-----------|----------|
| Мир с полосами и знаками вместо трассы | Клонировать `turtlebot3_simulations`, запускать `-World autorace`. |
| Езда по полосе + знаки + объезд препятствий | Запуск с `-World autorace -Autorace` (образ с turtlebot3_autorace и зависимостями). |
| Езда по полосе без выезда (подстройка) | Релей даёт bird's eye (IPM). При сбоях (Lane state: 0, уход в круг): выполните калибровку: запуск с `-UseCameraCalibration -LaneCalibration`, настройка в rqt, сохранение lane.yaml (см. раздел про калибровку). |
| Детекция знаков и решения | Запускать detect_sign (и при необходимости mission-узлы) с выбранной mission; согласовать топики с diff_robot. |

Ссылки:

- [TurtleBot3 Autonomous Driving (e-Manual)](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/)
- [turtlebot3_simulations (GitHub)](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_autorace (GitHub)](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)
