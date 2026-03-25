# Пакет `rtk2026_bringup` (сборка системы и сценарии запуска)

## Назначение

Содержит launch‑файлы, которые собирают воедино драйвер, базу, локализацию, SLAM, навигацию и периферию.  
Отвечает за сценарии «поднять весь робот», «только база», «база + SLAM + Nav2». Симуляция на трассе — отдельно в `rtk2026_simulation` (`rtk2026_diff_robot_track.launch.py`).

## Основные launch‑файлы

- `rtk2026.launch.py` — просмотр URDF и RViz (только описание робота).
- `rtk2026_driver_base.launch.py` — драйвер + база (реальный робот или fake encoder).
- `full.launch.py` — полный стек:
  - драйвер/база;
  - EKF (`rtk2026_localization`);
  - SLAM (`rtk2026_slam`);
  - Nav2 (`rtk2026_navigation`).

## Параметры

Типичные аргументы:

- `use_description` — включать ли `robot_state_publisher` и RViz;
- `use_fake_encoder` — использовать ли заглушку энкодера;
- `use_localization`, `use_slam`, `use_navigation` — включать ли соответствующие подсистемы;
- `use_sim_time` — время от симуляции (Gazebo и т.п.).

