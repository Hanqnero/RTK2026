# Пакет `rtk2026_simulation` (симуляция и интеграция с Isaac/Gazebo)

## Назначение

Содержит общие launch‑файлы и описание робота для симуляций:

- интеграция с Isaac Sim / Isaac Lab (через `simulation.launch.py`);
- новый Gazebo‑пайплайн с дифф‑приводом и `ros2_control` (`rtk2026_gazebo.launch.py`).

## Основные launch‑файлы

- `launch/simulation.launch.py`:
  - общий сценарий «симуляция + SLAM + Nav2» для Isaac/одометрии извне;
  - аргументы:
    - `use_sim_time` — использовать время симуляции;
    - `use_fake_scan` — включить `fake_scan` при отсутствии `LaserScan`;
    - `use_slam`, `use_navigation`, `use_localization`;
    - `use_rviz`, `rviz_config`;
    - параметры для `trigger_nav2_bringup` в one‑container режиме.
  - включает:
    - `rtk2026_localization` (EKF);
    - `rtk2026_slam` (slam_toolbox);
    - `rtk2026_navigation` (Nav2);
    - `rtk2026_peripherals` (fake_scan, TF, clock).
- `launch/rtk2026_gazebo.launch.py`:
  - Gazebo + URDF‑робот с `ros2_control`;
  - аргументы:
    - `world` — путь к `.world` внутри контейнера;
    - `x, y, z, R, P, Y` — начальная поза;
    - `use_sim_time`.
  - генерирует URDF из `urdf/rtk2026_diff_drive_gazebo.urdf.xacro` и `config/diff_drive_controller.yaml`;
  - стартует Gazebo, спаунит робота и активирует контроллеры `joint_state_broadcaster` и `diff_drive_controller`.

## URDF для симуляции

- `urdf/rtk2026_diff_drive_gazebo.urdf.xacro`:
  - дифф‑база с колёсами, фиксированным `base_footprint` и `ros2_control` описанием;
  - `libgazebo_ros2_control.so` и конфиг контроллера.

