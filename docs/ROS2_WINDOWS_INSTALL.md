# Docker‑образы RTK2026 и режимы запуска (Windows)

Этот документ описывает **именно Docker‑окружение** проекта RTK2026 на Windows: какие образы используются и как их запускать в разных режимах.  
Локальная установка ROS2 на Windows не обязательна — весь стек работает в контейнере.

## Базовый образ `rtk2026:latest`

### Сборка

Из корня репозитория (`RTK/RTK2026`):

```powershell
cd C:\CursorProject\Robotics\RTK\RTK2026
docker compose -f docker/docker-compose.yml build
```

или напрямую:

```powershell
docker build -t rtk2026:latest -f docker/Dockerfile .
```

Dockerfile:

- базируется на `ros:humble-ros-base-jammy`;
- устанавливает ROS‑пакеты: Nav2, slam_toolbox, gazebo_ros, ros2_control, ros2_controllers и т.д.;
- копирует `src/` в `/workspace/src`, запускает `colcon build --symlink-install`;
- настраивает автосорсинг `/opt/ros/humble/setup.bash` и `/workspace/install/setup.bash`;
- **entrypoint по умолчанию**:

```bash
ros2 launch rtk2026_bringup rtk2026.launch.py
```

### Режимы запуска

- **Интерактивный терминал (разработка)**:

  ```powershell
  docker run --rm -it rtk2026:latest bash
  ```

  Внутри:

  ```bash
  source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
  ros2 launch rtk2026_bringup rtk2026_driver_base.launch.py use_description:=false use_fake_encoder:=true
  ```

- **Bringup по умолчанию (без входа в контейнер)**:

  ```powershell
  docker run --rm rtk2026:latest
  ```

- **Любой launch по своему выбору**:

  ```powershell
  docker run --rm rtk2026:latest bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_bringup full.launch.py use_description:=false use_fake_encoder:=true use_slam:=true use_navigation:=true"
  ```

## Обёртки в `scripts/` (основные сценарии)

Все PowerShell‑скрипты запускаются из корня репозитория.

### `scripts/run_docker.ps1`

- Открывает интерактивный `bash` в контейнере `rtk2026:latest`.
- Удобен для ручного запуска `ros2`, `colcon`, тестов.

### `scripts/run_docker_simulation.ps1`

- Запускает `rtk2026_simulation/simulation.launch.py` с нужными флагами.
- Предназначен для сценариев с Isaac Lab (см. `docs/isaac/ISAAC_SIM_TEST.md`).

### `scripts/run_docker_isaac_slam.ps1` и `run_docker_slam*.ps1`

- Набор скриптов по мотивам zero‑to‑slam:
  - `run_docker_isaac_slam.ps1` — один контейнер: SLAM + Nav2 поверх `/odom` и `/scan` из Isaac Sim;
  - `run_docker_slam.ps1` — контейнер **base**: SLAM, fake_scan, clock, статические TF/карта;
  - `run_docker_slam_nav2.ps1` — контейнер только с Nav2;
  - `stop_docker_containers.ps1` — остановка всех контейнеров RTK2026.
- Детали и порядок запуска описаны в `docs/isaac/ISAAC_ROS_SLAM.md`.

## Полный симуляционный стек: Gazebo + SLAM + Nav2 + explorer

Для нового RTK‑пайплайна (Gazebo + SLAM + Nav2 + фронтир‑эксплорер) используется тот же образ `rtk2026:latest`, обёрнутый в `scripts/run_rtk2026_sim.ps1`.

### `scripts/run_rtk2026_sim.ps1`

**Назначение:** один скрипт запускает:

- Gazebo с дифф‑роботом RTK2026 (`rtk2026_simulation/rtk2026_gazebo.launch.py`);
- SLAM Toolbox + Nav2 с параметрами для SLAM;
- фронтир‑эксплорер `rtk2026_nav2_explorer/explorer` для автоматического исследования.

**Параметры:**

- `[switch]$Build` — пересобрать образ `rtk2026:latest` (через `docker/Dockerfile`);
- `[switch]$Explore` — включить/выключить эксплорер (флаг зарезервирован, сейчас сценарий одинаковый);
- `[string]$World = "city"` — выбор мира:
  - `city` → `/gazebo_worlds/worlds/small_city.world`;
  - `track` → `/gazebo_worlds/worlds/silverstone_track.world`;
  - любое другое значение — путь к `.world` внутри контейнера.

**Требования на хосте:**

- клонированный репозиторий миров Gazebo:

  ```text
  C:\CursorProject\Robotics\gazebo_models_worlds_collection
  ```

  он монтируется в контейнер как `/gazebo_worlds:ro`.

**Пример запуска:**

```powershell
cd C:\CursorProject\Robotics\RTK\RTK2026
.\scripts\run_rtk2026_sim.ps1 -Build -Explore -World city
```

В контейнере при этом выполняется:

```bash
ros2 launch rtk2026_bringup rtk2026_sim_slam_explore.launch.py world:=... x:=... y:=... z:=...
```

Launch‑файл `rtk2026_sim_slam_explore.launch.py`:

- поднимает Gazebo и спаунит URDF‑робота с `ros2_control`;
- через задержку запускает Nav2 + SLAM (`rtk2026_nav2_explorer/rtk2026_nav2_slam.launch.py`);
- ещё через задержку — фронтир‑эксплорер (`rtk2026_nav2_explorer/rtk2026_explorer.launch.py`).

## Нужен ли локальный ROS2

Если захочется отлаживать узлы RTK2026 **без Docker** (на Linux или Windows), смотри официальную документацию ROS2 Humble:  
`https://docs.ros.org/en/humble/`  
В рамках этого проекта основной и рекомендуемый путь — Docker: все рабочие сценарии (bringup, SLAM, Nav2, Gazebo, интеграция с Isaac) уже покрыты образами и скриптами, описанными здесь и в `DOCKER_WINDOWS.md` / `docs/isaac/*`.

# Установка ROS2 Humble на Windows

## Состояние Chocolatey

Скрипт установки Chocolatey может сообщать "An existing Chocolatey installation was detected", но путь при этом пустой ("at ''") — установка не завершена. Так бывает, если переменная `ChocolateyInstall` задана пустой или указывает на несуществующую папку.

Проверка: в PowerShell выполните `choco --version`. Если команда не найдена — используйте один из вариантов ниже.

### Если скрипт пишет "installation at ''" и не ставит Chocolatey

Выполните в **PowerShell от имени администратора** (чтобы сбросить переменные и при необходимости удалить пустую папку):

```powershell
# Удалить переменную ChocolateyInstall (она мешает переустановке)
[Environment]::SetEnvironmentVariable("ChocolateyInstall", $null, "User")
[Environment]::SetEnvironmentVariable("ChocolateyInstall", $null, "Machine")

# Удалить пустую или неполную папку (если есть)
if (Test-Path "C:\ProgramData\chocolatey") {
    $bin = "C:\ProgramData\chocolatey\bin\choco.exe"
    if (-not (Test-Path $bin)) { Remove-Item "C:\ProgramData\chocolatey" -Recurse -Force -ErrorAction SilentlyContinue }
}

# Закройте это окно, откройте новое PowerShell от имени администратора и снова запустите установку Chocolatey
```

После этого закройте терминал, откройте новый **PowerShell от имени администратора** и снова выполните установку Chocolatey (шаг 1 ниже).

---

## Вариант A: Chocolatey + Microsoft ROS (рекомендуется)

Требуется **PowerShell от имени администратора**.

### Шаг 1: Установить Chocolatey (если ещё нет)

Обязательно: **PowerShell запущен от имени администратора** (Win+X -> "Терминал (Администратор)" или правый клик по PowerShell -> "Запуск от имени администратора"). Без этого установка в `C:\ProgramData\chocolatey` не пройдёт.

```powershell
Set-ExecutionPolicy Bypass -Scope Process -Force
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072
iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
```

Закройте окно и откройте новое (можно обычное). Проверьте: `choco --version`.

### Шаг 2: Установить ROS2 Humble через Microsoft feed

В том же окне (администратор):

```powershell
choco source add -n=ros-win -s="https://aka.ms/ros/public" --priority=1
choco upgrade ros-humble-desktop -y --execution-timeout=0
```

Установка займёт много времени (десятки минут). По умолчанию ROS2 ставится в `C:\opt\ros\humble` (если использовался `ChocolateyInstall=c:\opt\chocolatey`) или в каталог Chocolatey.

### Шаг 3: Активация окружения

В новом терминале (можно обычном):

```batch
C:\opt\ros\humble\x64\setup.bat
```

Либо если ROS установился в другое место — укажите путь к `setup.bat` в установочной папке ROS2.

После этого доступны команды `ros2`, `colcon` и т.д.

### Автоматический скрипт

Из корня репозитория в **PowerShell от имени администратора**:

```powershell
cd c:\CursorProject\Robotics\RTK\RTK2026
.\scripts\install_ros2_windows.ps1
```

Скрипт добавляет источник ROS и ставит `ros-humble-desktop`; установку Chocolatey при необходимости нужно выполнить вручную (шаг 1 выше).

---

## Вариант B: Бинарный архив (без Chocolatey)

Если нет прав администратора или не хотите использовать Chocolatey:

1. Скачайте архив с [Releases ROS2](https://github.com/ros2/ros2/releases): `ros2-humble-*-windows-release-amd64.zip`.
2. Распакуйте в каталог, например `C:\dev\ros2_humble`.
3. Установите зависимости по [официальной инструкции](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html) (Python 3.8.3, Visual Studio 2019, OpenSSL, vcredist, Qt5, OpenCV и др.).
4. Активация в каждом новом терминале:

   ```batch
   call C:\dev\ros2_humble\local_setup.bat
   ```

---

## После установки ROS2

1. Сборка workspace RTK2026 (в терминале с активированным ROS2):

   ```batch
   cd c:\CursorProject\Robotics\RTK\RTK2026
   call C:\opt\ros\humble\x64\setup.bat
   colcon build --symlink-install
   call install\setup.bat
   ```

2. Запуск симуляции для Isaac Lab:

   ```batch
   ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_slam:=true
   ```

См. также [docs/isaac/ISAAC_SIM_TEST.md](isaac/ISAAC_SIM_TEST.md) и `scripts/run_isaac_sim_test.ps1`.
