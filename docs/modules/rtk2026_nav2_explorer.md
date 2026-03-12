# Пакет `rtk2026_nav2_explorer` (фронтир‑эксплорер для SLAM)

## Назначение

Автономное исследование карты поверх Nav2 + SLAM:

- подписка на `/map`;
- поиск фронтиров (границ между известной и неизвестной областью);
- выбор следующей целевой точки;
- отправка целей в Nav2 (`/navigate_to_pose`).

## Основные компоненты

- `rtk2026_nav2_explorer/explorer_node.py`:
  - `rclpy`‑нода с `ActionClient` к `nav2_msgs/action/NavigateToPose`;
  - использует TF (`map` → `base_link`) для определения текущей позы;
  - фильтрует цели (минимальная дистанция, чёрный список неудачных целей);
  - имеет режим «bootstrap» — первый прямой проезд для наращивания карты.
- `config/nav2_params_slam.yaml`:
  - параметры Nav2 для режима SLAM (rolling global costmap и т.д.).

## Launch‑файлы

- `launch/rtk2026_nav2_slam.launch.py`:
  - обёртка над `nav2_bringup/bringup_launch.py` с `slam:=True`;
  - использует `nav2_params_slam.yaml`.
- `launch/rtk2026_explorer.launch.py`:
  - запускает `explorer` с `use_sim_time`;
  - предполагает уже работающие Nav2 + SLAM.

