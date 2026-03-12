# Isaac Sim и ROS2 в Docker (по образцу zero-to-slam)

## Что и в каком порядке запускать

1. **Собрать образ (один раз)**  
   Из корня RTK2026:
   ```powershell
   cd C:\CursorProject\Robotics\RTK\RTK2026
   docker compose -f docker/docker-compose.yml build
   ```

2. **Запустить Isaac Sim**  
   Omniverse Launcher → Isaac Sim (или Isaac Lab). Загрузить сцену RTK2026 через скрипт `run_rtk2026_scene.py` (см. README в `isaac_lab/`), нажать **Play**. Скрипт сцены через ROS2‑мост публикует `/clock` и `/odom` (кадры `odom`, `base_link`), подписывается на `/cmd_vel` и передает команды роботу.

3. **Запустить ROS2‑стек в Docker**  
   В **отдельном** терминале, из корня RTK2026:
   ```powershell
   cd C:\CursorProject\Robotics\RTK\RTK2026
   .\scripts\run_docker_isaac_slam.ps1
   ```
   В контейнере поднимаются: `robot_state_publisher`, `odom_tf_broadcaster`, `fake_scan`, `slam_toolbox`, `Nav2`. Кадр `odom` и одометрия приходят из Isaac; если Isaac не публикует `/odom`, Nav2 будет ждать этот кадр.

4. **(По желанию) RViz на хосте**  
   В контейнере RViz не запускается (нет дисплея). Если на хосте установлены ROS2 Humble и пакеты RTK2026:
   ```powershell
   $env:ROS_DOMAIN_ID = "0"
   ros2 run rviz2 rviz2 -d C:\CursorProject\Robotics\RTK\RTK2026\src\rtk2026_description\rviz\rtk2026_sim_slam.rviz
   ```
   В RViz будут траектория `/odom`, карта `/map`, план `/plan`.

**Только SLAM, без Nav2:** `.\scripts\run_docker_isaac_slam.ps1 -NoNavigation`  
**Пересобрать образ:** `.\scripts\run_docker_isaac_slam.ps1 -Build`

---

## Режим «только Docker» (без Isaac)

Симулятор не запускать. Два контейнера (как в zero‑to‑slam):

```powershell
# Терминал 1 — base (SLAM, fake_scan, clock, static tf/map)
.\scripts\run_docker_slam.ps1

# Терминал 2 — только Nav2
.\scripts\run_docker_slam_nav2.ps1
```

Опции: `-Build` (пересборка образа), `-Rviz` (для base, с X‑сервером и DISPLAY на Windows).

**Дальше:**

1. Запустить base (терминал 1), затем Nav2 (терминал 2). В логах Nav2 дождаться `Managed nodes are active`.
2. Проверить Nav2 без GUI: в третьем терминале `.\scripts\send_nav_goal.ps1` (робот без симулятора не поедет, но в логах появятся план и `/cmd_vel`).
3. С RViz: `.\scripts\run_docker_slam.ps1 -Rviz` (предварительно поднять X‑сервер на Windows). В RViz: `/map` (Map), `/plan` (Path), 2D Goal Pose.

---

## Сравнение с zero‑to‑slam

В [zero‑to‑slam](https://github.com/Caian/zero-to-slam):

- Isaac Sim на хосте (Linux) с ROS2‑мостом публикует `/odom`, `/scan`, подписывается на `/cmd_vel`;
- ROS2 в контейнере (Podman), каждый компонент в отдельном контейнере;
- везде `--network host`, проброс `DISPLAY` и `/tmp/.X11-unix`; RViz в контейнере рисует на хосте.

У нас:

- **Один контейнер, один launch** для упрощения на Windows: `robot_state_publisher`, `odom_tf_broadcaster`, `fake_scan`, `slam_toolbox`, `Nav2`;
- источник `/odom` и `odom`‑кадра — Isaac Sim (мост); `odom_tf_broadcaster` ретранслирует TF из `/odom`;
- RViz обычно запускается **на хосте** с конфигом `rtk2026_sim_slam.rviz` и тем же `ROS_DOMAIN_ID`.

### Раздельные контейнеры (аналог zero‑to‑slam)

- **Терминал 1:** `.\scripts\run_docker_slam.ps1` — контейнер base: `robot_state_publisher`, `fake_scan`, `clock_publisher`, статические TF/карта, `slam_toolbox`.
- **Терминал 2:** `.\scripts\run_docker_slam_nav2.ps1` — контейнер только с Nav2.

Оба контейнера в сети `rtk2026_net` с одним `ROS_DOMAIN_ID`. Для DDS между контейнерами используется CycloneDDS (`rmw_cyclonedds_cpp`) и явные peer‑адреса.

---

## Конфиг RViz для симуляции

`src/rtk2026_description/rviz/rtk2026_sim_slam.rviz`:  
Fixed Frame `odom`, TF, RobotModel, Odometry (`/odom`), Map (`/map`), Path (`/plan`).

Запуск с хоста:

```bash
ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_slam:=true use_navigation:=true use_rviz:=true rviz_config:=rtk2026_sim_slam.rviz
```

---

## Полезные ссылки

- [zero‑to‑slam](https://github.com/Caian/zero-to-slam)
- [nvidia_isaac-sim_ros2_docker](https://github.com/arambarricalvoj/nvidia_isaac-sim_ros2_docker)
- [Isaac Sim ROS2 Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html)

