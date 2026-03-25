# Пакет `rtk2026_navigation` (Nav2 конфигурация)

## Назначение

Содержит конфигурацию и launch‑файлы для стека Nav2:

- локальный и глобальный costmap’ы;
- поведение планировщиков и контроллеров;
- настройки lifecycle / autostart.

## Основные компоненты

- `config/nav2_params.yaml` (и/или отдельные YAML для режимов):
  - параметры `bt_navigator`, `controller_server`, `planner_server`;
  - параметры `local_costmap` и `global_costmap` (размер, частота, rolling window и т.п.);
  - кадры `global_frame`, `robot_base_frame`, имена топиков `/cmd_vel`, `/scan`.
- `launch/navigation.launch.py`:
  - поднимает Nav2‑стек;
  - принимает параметры:
    - `use_sim_time`;
    - `autostart` (для one‑container сценариев);
    - путь к YAML с параметрами.

## Использование

- В реальном роботе — через `rtk2026_bringup/full.launch.py` с `use_navigation:=true`.
- В симуляции/SLAM — через `rtk2026_simulation/rtk2026_diff_robot_track.launch.py` (включает Nav2 через `rtk2026_nav2_explorer`) или отдельно `rtk2026_nav2_explorer/rtk2026_nav2_slam.launch.py`.

