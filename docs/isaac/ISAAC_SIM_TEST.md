# Тест RTK2026 в Isaac Sim / Isaac Lab

Пошаговая инструкция для проверки стека RTK2026 в симуляции Isaac Sim (или Isaac Lab) через `rtk2026_simulation/simulation.launch.py`.

## Обзор

1. **Терминал 1**: запуск Isaac Sim / Isaac Lab со сценой, где включён ROS2‑мост: публикуется `/clock`, `/odom`, TF `odom` → `base_link`, подписка на `/cmd_vel`. По желанию — публикация `/scan` или PointCloud2.
2. **Терминал 2**: запуск нашего ROS2‑стека с `use_sim_time:=true` (`simulation.launch.py` внутри контейнера).
3. **Терминал 3**: управление (`ros2 topic pub /cmd_vel ...`) или проверка топиков.

Убедись, что **ROS_DOMAIN_ID** совпадает у Isaac и контейнера (например, `0`).

---

## Вариант A: Isaac Sim standalone (готовые примеры Carter / TurtleBot)

Если установлен Isaac Sim с примерами:

1. Найти пример со встроенным ROS2‑мостом, например:
   - `standalone_examples/api/isaacsim.ros2.bridge/carter_stereo.py` (Carter: одометрия, PointCloud2, камеры, `cmd_vel`);
   - или пример с TurtleBot (URDF + drive‑tutorial).
2. Поднять окружение ROS2 согласно документации Isaac Sim (Install ROS 2 / Using Terminal).
3. Запустить пример (из каталога Isaac Sim):
   ```batch
   python.bat standalone_examples\api\isaacsim.ros2.bridge\carter_stereo.py
   ```
   На Linux — `python.sh`.
4. Дождаться загрузки сцены и нажать **Play**.
5. Во втором терминале (где доступен наш workspace):
   ```bash
   export ROS_DOMAIN_ID=0
   source install/setup.bash
   ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_slam:=true
   ```
   `use_fake_scan:=true` нужен, если в сцене нет `LaserScan` (`/scan`), а есть только `PointCloud2` — тогда мы публикуем заглушку `/scan` для SLAM.
6. Проверка:
   ```bash
   ros2 topic list
   ros2 topic echo /odom --once
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
   ```

---

## Вариант B: Isaac Lab (собственная сцена без готового ROS2‑моста)

### Запуск Isaac Lab и нашего стека (через Docker)

1. **Терминал 1** — запустить Isaac Lab со сценой RTK2026 (пол, свет, при желании робот). Если расширение ROS2 включено, будет публиковаться `/clock`:
   ```powershell
   cd <RTK2026_root>
   .\scripts\run_isaac_lab.ps1
   ```
   На Linux/WSL: `./scripts/run_isaac_lab.sh`. Нужен установленный Isaac Lab рядом с RTK (например, `.../IsaacLab`) и активированное окружение (conda/env).
2. Дождаться открытия окна симулятора и нажать **Play**.
3. **Терминал 2** — запустить ROS2‑стек в Docker:
   ```powershell
   .\scripts\run_docker_simulation.ps1
   ```
   Скрипт запускает `simulation.launch.py` с `use_sim_time:=true use_fake_scan:=true use_slam:=true`.  
   Если в Isaac Lab нет ROS2‑моста, `/clock` не придёт — узлы будут ждать время; при корректном мосте `/clock` появится и время синхронизируется.

Isaac Lab может запускать только «голую» сцену (пол, свет, без робота и без моста). В этом случае есть два пути.

### B1. Добавить ROS2‑мост в Isaac вручную

После запуска сцены открыть в Isaac Sim **Window → Graph Editors → Action Graph** и добавить узлы:
- **ROS2 Context** (`domain_id = 0`);
- **ROS2 Publish Clock** (топик `/clock`);
- **ROS2 Subscribe Twist** (топик `/cmd_vel`);
- **ROS2 Publish Odometry** (топик `odom`, кадры `odom` → `base_link`);
- **Differential Controller** + **Articulation Controller** (если в сцене есть дифф‑робот) и связать их с `Subscribe Twist` и сочленениями робота.

Сцену сохранить — в следующий раз ROS2‑мост поднимется автоматически.

### B2. Только наш стек + fake `/scan` (без робота в Isaac)

Если в Isaac запущена простая сцена без робота и без ROS2‑моста:
- запускать `simulation.launch.py` с `use_sim_time:=true use_fake_scan:=true`;
- одометрия и `/scan` будут заглушками (или их не будет совсем). Для полноценной навигации нужны `/odom` и `/scan` от симулятора; но для быстрой проверки запуска узлов и топиков этого достаточно.

---

## Параметры `simulation.launch.py`

| Аргумент         | Default | Описание |
|------------------|---------|---------|
| `use_sim_time`   | `true`  | Обязательно `true` для Isaac Sim / Isaac Lab. |
| `use_fake_scan`  | `false` | `true` — запустить `fake_scan` (топик `/scan`), если сим не даёт `LaserScan`. |
| `use_slam`       | `false` | Запустить `slam_toolbox` (нужен `/scan`). |
| `use_navigation` | `false` | Запустить Nav2 (нужны карта и `/scan`). |
| `use_localization` | `false` | Запустить EKF. |
| `use_rviz`       | `false` | Запустить RViz2 внутри контейнера (обычно выключено на Windows). |

Актуальный список аргументов и их значения можно посмотреть в `rtk2026_simulation/launch/simulation.launch.py`.

---

## Краткий чек‑лист теста

1. Isaac Sim / Isaac Lab запущен, сцена в режиме **Play**, ROS2‑мост (bridge) включён.
2. `ROS_DOMAIN_ID` одинаков у симулятора и контейнера (например, `0`).
3. `ros2 topic list` показывает `/clock`, `/odom`, `/cmd_vel` (и по желанию `/scan`).
4. `ros2 topic echo /odom --once` даёт сообщение с `header.frame_id=odom`, `child_frame_id=base_link`.
5. После `ros2 topic pub /cmd_vel ...` робот в симуляции движется (или как минимум меняется одометрия).
6. При `use_slam:=true` `slam_toolbox` получает `/scan` и `/odom` и строит карту.

---

## Типовые проблемы

- **Нет топиков** — проверить `ROS_DOMAIN_ID`, что Isaac запущен, нажата кнопка **Play** и расширение ROS2‑моста включено.
- **Нет `/clock`** — Isaac работает без ROS2‑моста; можно:
  - либо включить мост (см. `ROS2_ISAAC_WINDOWS.md`),  
  - либо запускать `simulation.launch.py` с `use_fake_scan` и не завязываться на сим‑время.
- **Нет `/scan`** — если в сцене только `PointCloud2`, использовать `use_fake_scan:=true` или узел `pointcloud_to_laserscan` в ROS2.

