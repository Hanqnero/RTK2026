# RTK2026: трассовая симуляция на macOS (M1/M2/M3) через TigerVNC

Запуск diff_robot на трассе Silverstone (SLAM + Nav2 + explorer) на Mac с чипом Apple Silicon. Один скрипт: сборка Docker-образа, контейнер с VNC, сборка пакетов и запуск симуляции. В VNC отображается только RViz; Gazebo работает в headless-режиме.

## 1. Предварительные условия

- macOS на M1/M2/M3.
- Установлен **Docker Desktop** (должен быть запущен перед скриптом).
- Установлен **TigerVNC Viewer**:

```bash
brew install tigervnc-viewer
```

- Репозиторий RTK2026, внутри которого присутствует каталог `ros-humble-desktop-m1_2-mac/` (базовый образ с VNC).

## 2. Запуск

```bash
cd /path/to/RTK2026
chmod +x scripts/run_rtk2026_diff_robot_mac_vnc.sh
./scripts/run_rtk2026_diff_robot_mac_vnc.sh
```

Что делает скрипт:

1. Проверка Docker -- если демон недоступен, завершается с ошибкой.
2. Сборка образа `ros-humble-desktop:latest` (amd64, эмуляция) из `ros-humble-desktop-m1_2-mac/`.
3. Запуск контейнера `rtk2026_desktop` в фоне: supervisord (Xvfb + VNC) + `sleep infinity`.
4. `colcon build` пакетов `rtk2026_simulation` и `diff_robot`, затем `ros2 launch`.

Подключение: **TigerVNC Viewer** -> `localhost:5900`.

Остановка:
- `Ctrl+C` -- остановка launch (контейнер остается).
- `docker rm -f rtk2026_desktop` -- полная остановка.

## 3. Суть решения

- **Платформа:** образ собирается под `linux/amd64` (эмуляция Rosetta).
- **Графика:** Xvfb + x11vnc; на Mac подключаемся по VNC к порту 5900 (RViz).
- **Gazebo:** запускается `gzserver` (headless), без gzclient.
- **Пути:** URDF и world ищутся через `get_package_share_directory("diff_robot")`, работает при любом расположении workspace.
- **RMW:** `rmw_fastrtps_cpp`.

## 4. Переменные в скрипте

В начале `scripts/run_rtk2026_diff_robot_mac_vnc.sh`:

- `ROOT` -- путь к каталогу RTK2026 на хосте.
- `USERNAME` -- имя пользователя в образе.
- `USERID`, `GROUPID` -- uid/gid (501/20 для staff на Mac).

После изменений достаточно перезапустить скрипт.
