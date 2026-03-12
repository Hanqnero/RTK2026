# ROS2 и Isaac Sim / Isaac Lab на Windows

## Зачем ROS2 в сцене

Сцена RTK2026 в Isaac Lab может публиковать `/clock` (simulation time), чтобы стек ROS2 в Docker работал в одном времени с симуляцией. Без моста можно использовать `use_fake_scan` и не зависеть от `/clock`.

## Как включить ROS2 Bridge в Isaac Sim

См. документацию Isaac Sim 4.5:  
<https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.ros2.bridge/docs/index.html>

1. **Расширение:** `isaacsim.ros2.bridge` (Isaac Sim 4.x) или `omni.isaac.ros2_bridge` (5.x).
2. **Включение при запуске:**
   - через GUI: после запуска сцены Window → Extensions, найти `isaacsim.ros2.bridge`, включить, при необходимости перезапустить;
   - через `isaac-sim.bat`:  
     ```bat
     isaac-sim.bat --enable isaacsim.ros2.bridge
     ```
3. **ROS2‑библиотеки:** перед запуском Isaac Sim:
   - либо активировать окружение ROS2 (например, Humble в WSL2),
   - либо использовать встроенные lightweight ROS2‑библиотеки Isaac Sim (по умолчанию fallback на `humble`).

## Если расширение не находится

В логе сцены:

```text
[RTK2026] No ROS2/bridge extensions found. Install Isaac Sim with ROS2 Bridge or use Docker stack without /clock.
```

Причины:

- Isaac Sim установлен без ROS2 Bridge (варианты pip‑установки);
- другая версия Isaac Sim (в 5.x расширение может называться `omni.isaac.ros2_bridge`);
- сочетание Windows + ROS2 (часто используют ROS2 в WSL2/VM, см. форум NVIDIA).

## Работа без ROS2 Bridge

- Запускать симуляцию как обычно (`run_isaac_lab.bat`), нажимать Play.
- ROS2‑стек в Docker запускать с `use_fake_scan` и без опоры на `/clock` от Isaac Sim.
- Так можно разрабатывать и тестировать навигацию и драйвер без полноценного ROS2‑моста.

## Полезные ссылки

- [ROS 2 Bridge — Isaac Sim 4.5](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.ros2.bridge/docs/index.html)
- [Isaac Sim on Windows 11 + ROS2 Bridge](https://forums.developer.nvidia.com/t/can-isaac-sims-ros2-bridge-be-used-on-windows-11/290937)

