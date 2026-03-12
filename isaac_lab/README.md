# Сцена Isaac Lab для RTK2026

Сцена для симуляции RTK2026 в Isaac Lab. Скрипт `run_rtk2026_scene.py` при наличии расширений ROS2-моста создает OmniGraph, который публикует `/clock` и `/odom` (кадры `odom`, `base_link`) и подписывается на `/cmd_vel`, передавая команды роботу. Для этого нужны расширения `omni.isaac.ros2_bridge` (или `isaacsim.ros2.bridge`) и для привода от `/cmd_vel` — `omni.isaac.wheeled_robots` (или `isaacsim.robot.wheeled_robots`).

## Требования

- Установленные Isaac Sim и Isaac Lab (внутри Isaac Sim).
- ROS2 Humble в отдельной сессии для запуска `rtk2026_simulation`.
- Энкодеры на роботе: MT6701 (магнитный модуль), см. `docs/ENCODER_MT6701.md`.

## Запуск сцены

**Linux / macOS** — из каталога Isaac Lab:

```bash
./isaaclab.sh -p /path/to/RTK2026/isaac_lab/run_rtk2026_scene.py --num_envs 1
```

**Windows (PowerShell):** из корня Isaac Lab вызовите скрипт с путём к сцене; либо из корня RTK2026: `.\scripts\run_isaac_lab.ps1` (пути к Isaac Lab и сцене подставляются автоматически, ожидается Isaac Lab рядом с каталогом RTK).

```powershell
cd <IsaacLab_root>
.\isaaclab.bat -p <RTK2026_root>\isaac_lab\run_rtk2026_scene.py --num_envs 1
```

Тест «проехать 1 м» (робот RTK2026, одометрия и тики из конфига rtk2026_base):

```powershell
.\isaaclab.bat -p <RTK2026_root>\isaac_lab\run_rtk2026_drive_1m.py --robot rtk2026
```

Поверхность пола: по умолчанию сетка; для пола как фанера (лаборатория) добавьте `--ground plywood`.

**Кинематическая база (как в Gazebo):** в `run_rtk2026_drive_1m.py` по умолчанию включён режим `--kinematic_base`: позиция и скорость базы задаются по одометрии и команде (cmd_vel), без физики опрокидывания — робот стоит на плоскости. Отключить: `--no_kinematic_base` (тогда база в полной физике, может заваливаться). Для управления по ROS2 cmd_vel тот же подход: база обновляется из odom + cmd_vel.

## Связь с ROS2

1. Запустите симуляцию (скрипт выше).
2. В другой сессии: `ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true`
3. Мост (расширение ROS2 в Isaac Sim или внешний узел) должен:
   - Публиковать: `/odom` (nav_msgs/Odometry), tf `odom` -> `base_link`
   - Публиковать: `/scan` (sensor_msgs/LaserScan) при наличии лидара в сцене
   - Подписываться: `/cmd_vel` (geometry_msgs/Twist)

Подробности: `src/rtk2026_simulation/docs/ISAAC_LAB.md`. Чтобы параллельно симуляции видеть в ROS-окне траекторию по одометрии, карту SLAM и план Nav2 (как в [zero-to-slam](https://github.com/Caian/zero-to-slam)), запускайте ROS2-стек в Docker: `.\scripts\run_docker_isaac_slam.ps1` (см. `docs/ISAAC_ROS_SLAM.md`).

## Где смотреть Action Graph робота (совпадение с ROS-узлами)

Action Graph в Isaac Sim — это OmniGraph, который связывает ROS2-топики с приводом и одометрией. Он должен соответствовать нашему ROS2-стеку (cmd_vel, odom, base_link).

**Где открыть:** в Isaac Sim после запуска сцены: **Window -> Graph Editors -> Action Graph**.

**Узлы, которые должны совпадать с ROS:**

| Узел в Action Graph | ROS2 | Наш стек |
|---------------------|------|----------|
| ROS2 Subscribe Twist (топик `/cmd_vel`) | geometry_msgs/Twist | base_controller подписывается на `cmd_vel_topic` (по умолчанию `cmd_vel`) |
| Differential Controller | преобразует Twist в скорости колёс | base_controller: Twist -> motor_command (PWM), по одометрии — wheel_separation, ticks_per_meter |
| Articulation Controller | прим робота, joint velocity/position | в симуляции — прим робота (Jackal или RTK2026 URDF) |
| ROS2 Publish Odometry (топик `odom`, frame_id odom, child_frame_id base_link) | nav_msgs/Odometry | base_controller публикует odom из энкодеров; в симе — из позиции/скорости прима |

Скрипт `run_rtk2026_scene.py` при запуске создает графы OmniGraph для `/clock`, `/odom` (Isaac Compute Odometry -> ROS2 Publish Odometry) и `/cmd_vel` (ROS2 Subscribe Twist -> Differential Controller -> Articulation Controller). Управление из ROS2 (`ros2 topic pub /cmd_vel`, Nav2) работает без ручного добавления узлов, если установлены расширения ROS2-моста и wheeled_robots. Скрипт `run_rtk2026_drive_1m.py` управляет роботом напрямую из Python (тест «1 м» по одометрии), без ROS2.

**Сохранение:** при ручном изменении графа в Isaac Sim сохраните сцену (Save) — граф запишется в USD.

## Gazebo (дифпривод по образцу ros2_diff_drive_robot)

Вариант симуляции в Gazebo Classic с ros2_control и DiffDriveController (как в [gurselturkeri/ros2_diff_drive_robot](https://github.com/gurselturkeri/ros2_diff_drive_robot)): робот управляется по `/cmd_vel`, одометрия и tf из контроллера.

- URDF: `src/rtk2026_simulation/urdf/rtk2026_diff_drive_gazebo.urdf.xacro` (колёса left/right_wheel_joint, wheel_separation 0.25 м, radius 0.06 м).
- Конфиг: `src/rtk2026_simulation/config/diff_drive_controller.yaml`.
- Запуск (после `colcon build` и установки пакетов Gazebo/ros2_control):

```bash
ros2 launch rtk2026_simulation gazebo_diff_drive.launch.py
```

Требуются: `ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-ros2-control`, `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-diff-drive-controller`, `ros-humble-joint-state-broadcaster`. При необходимости GUI: в отдельном терминале `ros2 launch gazebo_ros gzclient.launch.py`.
