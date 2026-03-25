# RTK2026: тест RTK-навигации на трассе (diff_robot)

**Симуляция:** RTK-стек (SLAM + Nav2 + explorer) на diff-роботе на трассе `silverstone_track.world`.

**Пакет `diff_robot`** — пример симуляционной платформы (не код RTK). Используется только для тестов навигации на трассе.

---

## Симуляция: diff_robot + RTK-стек на трассе

### Требования

- Docker Desktop (Windows) или Docker (Linux/Mac).
- X-сервер для RViz (Windows: VcXsrv/Xming; Mac: TigerVNC, см. [RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md](RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md)).
- Опционально: коллекция миров `gazebo_models_worlds_collection` для мира `city`.

### Запуск на Windows (PowerShell)

```powershell
cd C:\path\to\RTK2026

# Первый запуск (пересборка образа):
.\scripts\run_rtk2026_diff_robot.ps1 -Build -Explore -World track

# Повторный запуск:
.\scripts\run_rtk2026_diff_robot.ps1 -Explore -World track
```

Параметры:

- `-Build` -- пересобрать образ из `docker/windows/Dockerfile`.
- `-Explore` -- включить фронтир-эксплорер.
- `-World`: `track` (Silverstone), `office`, `city`, или произвольный путь к `.world`.

### Запуск на macOS (M1/M2/M3)

```bash
cd /path/to/RTK2026
./scripts/run_rtk2026_diff_robot_mac_vnc.sh
```

Подключение: TigerVNC -> `localhost:5900`. Подробно: [RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md](RTK_DIFF_TRACK_SIM_MAC_TIGERVNC.md).

### Запуск на Linux (Bash)

```bash
cd ~/path/to/RTK2026
docker build -t rtk2026:latest -f docker/windows/Dockerfile .

docker run --rm \
  -e ROS_DOMAIN_ID=0 \
  -e DISPLAY=${DISPLAY:-:0} \
  -v /path/to/gazebo_models_worlds_collection:/gazebo_worlds:ro \
  --name rtk2026_diff_robot_gazebo \
  rtk2026:latest \
  bash -lc "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
            ros2 launch rtk2026_simulation rtk2026_diff_robot_track.launch.py \
                 world:=/workspace/src/diff_robot/world/silverstone_track.world \
                 use_sim_time:=true explore:=true"
```
