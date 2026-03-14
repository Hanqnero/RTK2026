# RTK2026: трассовая симуляция на macOS (M1/M2) через TigerVNC

Запуск примера diff_robot на трассе Silverstone (SLAM + Nav2 + explorer) на Mac с чипом M1/M2/M3. Используется один скрипт: сборка Docker-образа, контейнер с VNC, сборка пакетов и запуск симуляции. В VNC отображается только окно RViz с картированием; Gazebo работает в headless-режиме (без окна).

## 1. Предварительные условия

- macOS на M1/M2/M3.
- Установлен **Docker Desktop** (должен быть запущен перед скриптом).
- Установлен **TigerVNC Viewer**:

```bash
brew install tigervnc-viewer
```

- Репозиторий RTK2026 на хосте, например:

```text
/Users/kamilishakov/CursorProjects/ROBO-RTK/RTK2026
```

- Внутри `RTK2026` присутствует клон репозитория **ros-humble-desktop-m1_2-mac** (образ с ROS2 Humble Desktop, Gazebo, RViz, Xvfb, x11vnc).

## 2. Запуск скрипта

Скрипт лежит в репозитории и уже готов к запуску:

```bash
cd /Users/kamilishakov/CursorProjects/ROBO-RTK/RTK2026
chmod +x scripts/run_rtk2026_diff_robot_mac_vnc.sh   # один раз, если ещё не исполняемый
./scripts/run_rtk2026_diff_robot_mac_vnc.sh
```

Что делает скрипт:

1. **Проверка Docker** — если демон недоступен, выводит сообщение и завершается.
2. **Сборка образа** — `docker buildx build --platform linux/amd64` образа `ros-humble-desktop:latest` из каталога `ros-humble-desktop-m1_2-mac` (с нужным пользователем и группой).
3. **Запуск контейнера** — контейнер `rtk2026_desktop` стартует в фоне с:
   - примонтированным `RTK2026` в `/home/kamilishakov/RTK2026`;
   - супервизором (Xvfb, x11vnc, clipboard) и командой `sleep infinity`, чтобы контейнер не завершался;
   - портом 5900 для VNC.
4. **Сборка и launch внутри контейнера** — через `docker exec` выполняется `colcon build` по пакетам `rtk2026_simulation` и `diff_robot`, затем `ros2 launch rtk2026_simulation rtk2026_diff_robot_track.launch.py` с `use_sim_time:=true`, `explore:=true`, `use_rviz:=true`.

После запуска скрипта откройте **TigerVNC Viewer** и подключитесь к:

```text
localhost:5900
```

В VNC будет видно только окно **RViz** с картированием (карта, робот, Nav2). Окно Gazebo не открывается — симуляция идёт через `gzserver` в headless-режиме.

Остановка: `Ctrl+C` в терминале (где запущен скрипт) останавливает `ros2 launch` и завершает `docker exec`; контейнер при этом продолжает работать. Чтобы остановить и контейнер:

```bash
docker rm -f rtk2026_desktop
```

## 3. Суть решения на Mac

- **Платформа**  
  На M1/M2 образ собирается под **linux/amd64** (эмуляция). Нативный arm64-образ без Gazebo в этом сценарии не используется.

- **Графика**  
  В контейнере нет реального дисплея. Запускаются **Xvfb** (виртуальный X) и **x11vnc**; с Mac подключаемся по VNC к порту 5900 и видим один X-десктоп (RViz). TigerVNC поддерживает подключение без пароля, что удобно для локального запуска.

- **Контейнер не падает**  
  Базовый образ запускает `start.sh` (supervisord + bash). В скрипте контейнер стартует с `--entrypoint ""` и командой `sudo supervisord & sleep infinity`: supervisord поднимает Xvfb и VNC в фоне, а главный процесс — `sleep infinity`, поэтому контейнер остаётся живым для `docker exec`.

- **Пути не зависят от /workspace**  
  Код монтируется в `/home/kamilishakov/RTK2026`. В launch-файле пути к URDF и world задаются через **get_package_share_directory("diff_robot")**; пакет `diff_robot` устанавливает каталоги **urdf** и **world** в `share` (см. CMakeLists.txt), поэтому ресурсы находятся и при таком раскладе.

- **Мир и меши**  
  В world-файле трассы используется относительный путь к мешу (`file://models/silverstone_track/...`); Gazebo запускается с рабочей директорией в каталоге world, чтобы этот путь разрешался корректно.

- **Только картирование в VNC**  
  Запускается **gzserver** (без gzclient), поэтому окно Gazebo не показывается; в VNC отображается только RViz с картой и визуализацией SLAM/Nav2/explorer.

- **RMW**  
  В контейнере задаётся `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` для совместимости с этой связкой.

## 4. Переменные в скрипте

При другом имени пользователя или пути к проекту отредактируйте в начале `scripts/run_rtk2026_diff_robot_mac_vnc.sh`:

- `ROOT` — путь к каталогу RTK2026 на хосте;
- `USERNAME` — имя пользователя в образе (и путь `/home/<USERNAME>/RTK2026` в контейнере);
- `USERID`, `GROUPID` — uid/gid для совместимости с монтированием (например 501 и 20 для staff на Mac).

После изменения `ROOT` или структуры репозитория достаточно перезапустить скрипт; образ пересоберётся при изменении `ros-humble-desktop-m1_2-mac`, а `colcon build` выполняется при каждом запуске (инкрементально).
