# Пакет `rtk2026_simulation`

## Назначение

Один сценарий: **diff_robot на трассе Silverstone** с полным RTK-стеком — SLAM (`slam_toolbox`), Nav2 и опционально frontier-explorer (`rtk2026_nav2_explorer`). Модель и мир Gazebo приходят из пакета `diff_robot`.

## Launch

- `launch/rtk2026_diff_robot_track.launch.py` — Gazebo, `robot_state_publisher` по URDF `diff_robot`, затем EKF, SLAM, Nav2 и при `explore:=true` — explorer. Основные аргументы: `world` (путь к `.world`), `use_sim_time`, `explore`, `use_rviz`, `use_gazebo_gui`, стартовая поза `x`/`y`/`z`/`R`/`P`/`Y`, `rvizconfig`.

Запуск из корня workspace после `source install/setup.bash`:

```bash
ros2 launch rtk2026_simulation rtk2026_diff_robot_track.launch.py
```

Подробнее по Docker и скриптам: [RTK_DIFF_TRACK_SIM.md](../RTK_DIFF_TRACK_SIM.md).
