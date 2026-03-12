# RTK2026: тест RTK‑навигации на трассе и отладка реального робота

Этот документ описывает два сценария:

- **Симуляция:** RTK‑стек (SLAM + Nav2 + explorer) на diff‑роботе из `ros2_diff_drive_robot` на трассе `silverstone_track.world`.
- **Отладка реального робота:** отдельный Docker‑контейнер на ПК, который поднимает SLAM + Nav2 + explorer + RViz + teleop поверх топиков от Raspberry Pi.

Все команды приводятся для **Windows/PowerShell** и для **Linux/Bash**.

**Примечание:** пакет `diff_robot` — это не код RTK, а пример симуляционной платформы (URDF и миры в стиле `ros2_diff_drive_robot`). Он используется только для тестов навигации на трассе; ядро проекта — пакеты `rtk2026_*`.

---

## 1. Симуляция: diff_robot + RTK‑стек на трассе

### Требования

- Клонированный репозиторий RTK2026 (этот проект).
- На хосте:
  - Docker Desktop (Windows) или Docker/Podman (Linux).
  - Клонированная коллекция миров для Gazebo:

    ```text
    C:\CursorProject\Robotics\gazebo_models_worlds_collection
    ```

    Для Linux путь аналогичный (например `/home/user/gazebo_models_worlds_collection`), но его нужно подставить в скрипт или в `docker run`.

- Запущенный X‑сервер (на Windows: VcXsrv/Xming) с разрешением подключений от Docker (`DISPLAY=host.docker.internal:0.0`).

### Что делает сценарий

Скрипт `scripts/run_rtk2026_diff_robot.ps1` внутри контейнера запускает:

- `rtk2026_simulation/rtk2026_diff_robot_track.launch.py`, который:
  - запускает Gazebo с миром `silverstone_track.world`;
  - спаунит URDF diff‑робота (`diff_robot/urdf/diff_robot.urdf`);
  - запускает `robot_state_publisher` и RViz с конфигом `diff_robot/urdf/rviz.rviz`;
  - после прогрева включает:
    - `rtk2026_nav2_explorer/rtk2026_nav2_slam.launch.py` (SLAM Toolbox + Nav2 c RTK‑параметрами),
    - `rtk2026_nav2_explorer/rtk2026_explorer.launch.py` (фронтир‑эксплорер).

Таким образом в Gazebo едет diff‑робот, а SLAM/навигация/эксплорер — из RTK‑кода.

### Запуск на Windows (PowerShell)

Из корня RTK2026:

```powershell
cd C:\CursorProject\Robotics\RTK\RTK2026

# Первый запуск (или после изменений) — пересборка образа:
.\scripts\run_rtk2026_diff_robot.ps1 -Build -Explore -World track

# Дальше можно без -Build:
.\scripts\run_rtk2026_diff_robot.ps1 -Explore -World track
```

Параметры:

- `-Build` — пересобрать образ `rtk2026:latest` по `docker/Dockerfile`;
- `-Explore` — включить RTK‑фронтир‑эксплорер (иначе запустится только SLAM + Nav2);
- `-World`:
  - `track` → `/workspace/src/diff_robot/world/silverstone_track.world`;
  - `office` → `/workspace/src/diff_robot/world/office_small.world`;
  - `city` → `/gazebo_worlds/worlds/small_city.world`;
  - любое другое значение — путь к `.world` внутри контейнера.

### Запуск на Linux (Bash, без PowerShell)

Эквивалентные шаги:

```bash
cd ~/path/to/RTK2026  # путь к репозиторию

# 1. Собрать образ
docker build -t rtk2026:latest -f docker/Dockerfile .

# 2. Запустить контейнер с diff_robot + RTK стеком
docker run --rm \
  -e ROS_DOMAIN_ID=0 \
  -e DISPLAY=${DISPLAY:-:0} \
  -v /home/user/gazebo_models_worlds_collection:/gazebo_worlds:ro \
  --name rtk2026_diff_robot_gazebo \
  rtk2026:latest \
  bash -lc "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
            ros2 launch rtk2026_simulation rtk2026_diff_robot_track.launch.py world:=/workspace/src/diff_robot/world/silverstone_track.world use_sim_time:=true explore:=true"
```

Отличия:

- путь к `gazebo_models_worlds_collection` нужно заменить на свой;
- переменная `DISPLAY` должна указывать на X‑сервер (например `:0`).

---

## 2. Отладочный Docker‑контейнер для реального робота

Цель: когда код RTK будет перенесён на Raspberry Pi (драйвер/база/датчики), на **ПК в Docker‑контейнере** можно запускать:

- SLAM Toolbox;
- Nav2;
- RTK‑explorer;
- RViz (карта, локализация, траектория);
- teleop с клавиатуры (`teleop_twist_keyboard`).

### Предпосылки

На **роботе (Raspberry Pi)**:

- поднят драйвер/база/лидар, публикуются:
  - `/odom` (TF `odom -> base_link`);
  - `/scan` (2D лидар);
  - дополнительные датчики по желанию.
- `ROS_DOMAIN_ID` совпадает с хостом/контейнером (например `0`);
- DDS (CycloneDDS/FastDDS) настроен так, чтобы контейнер на ПК видел топики робота (обычно достаточно одной сети и одинакового `ROS_DOMAIN_ID`).

На **ПК**:

- установлен Docker Desktop (Windows) или Docker (Linux);
- включён X‑сервер для RViz.

### Что делает скрипт `run_rtk2026_robot_debug.ps1`

Внутри контейнера:

- запускает:

  ```bash
  ros2 launch rtk2026_simulation simulation.launch.py \
       use_sim_time:=false \
       use_fake_scan:=false use_fake_odom:=false \
       use_slam:=true use_navigation:=true use_localization:=true \
       use_rviz:=true rviz_config:=rtk2026_sim_slam.rviz
  ```

  — это SLAM Toolbox + Nav2 + EKF + RViz, ожидающие `/odom` и `/scan` от робота;

- через ~20 секунд (когда Nav2 поднялся) при `-Explore` запускает:

  ```bash
  ros2 launch rtk2026_nav2_explorer rtk2026_explorer.launch.py use_sim_time:=false
  ```

- в том же контейнере запускает:

  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

  — источник `/cmd_vel` для ручного управления с клавиатуры.

### Запуск на Windows (PowerShell)

Из корня RTK2026:

```powershell
cd C:\CursorProject\Robotics\RTK\RTK2026

# Первый раз: пересобрать образ с teleop_twist_keyboard
.\scripts\run_rtk2026_robot_debug.ps1 -Build -Explore

# Дальше — без -Build
.\scripts\run_rtk2026_robot_debug.ps1 -Explore
```

Параметры:

- `-Build` — пересобрать образ `rtk2026:latest`;
- `-Explore` — добавить RTK‑explorer поверх SLAM + Nav2.

Важно:

- `ROS_DOMAIN_ID` в скрипте по умолчанию `0`. Если на роботе другой — задай ту же переменную в PowerShell перед запуском:

  ```powershell
  $env:ROS_DOMAIN_ID = "10"
  .\scripts\run_rtk2026_robot_debug.ps1 -Explore
  ```

### Запуск на Linux (Bash)

Эквивалентные шаги:

```bash
cd ~/path/to/RTK2026

docker build -t rtk2026:latest -f docker/Dockerfile .

export ROS_DOMAIN_ID=0  # тот же, что на роботе

docker run --rm \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -e DISPLAY=${DISPLAY:-:0} \
  --name rtk2026_robot_debug \
  rtk2026:latest \
  bash -lc "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
            ros2 launch rtk2026_simulation simulation.launch.py \
                 use_sim_time:=false use_fake_scan:=false use_fake_odom:=false \
                 use_slam:=true use_navigation:=true use_localization:=true \
                 use_rviz:=true rviz_config:=rtk2026_sim_slam.rviz & \
            sleep 20 && \
            ros2 launch rtk2026_nav2_explorer rtk2026_explorer.launch.py use_sim_time:=false & \
            ros2 run teleop_twist_keyboard teleop_twist_keyboard; wait"
```

### Логи и запись данных

Внутри отладочного контейнера (как для симуляции, так и для реального робота) можно записывать бэги:

```bash
ros2 bag record /odom /tf /tf_static /scan /map /cmd_vel -o rtk2026_debug_bag
```

Либо через дополнительный терминал/контейнер с тем же образом и `ROS_DOMAIN_ID`. Далее бэги можно анализировать оффлайн (SLAM, Nav2, explorer) без участия робота.

