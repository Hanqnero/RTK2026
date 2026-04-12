# Работа с ROS2 через Docker (Windows)

На Windows не требуется устанавливать ROS2 локально: весь стек запускается в контейнере.

## Требования

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) для Windows
- Клонированный репозиторий RTK2026

**Использование видеокарты (GPU) для Gazebo:** чтобы симуляция рендерилась на GPU, нужны Docker на базе WSL2 и [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) в WSL2. Тогда при запуске скрипта diff_robot добавьте флаг `-UseGpu` (контейнер запустится с `--gpus all`). Без этого Gazebo использует программный рендер.

## Сборка образа

```powershell
cd C:\path\to\RTK2026
docker build -t rtk2026:latest -f docker/windows/Dockerfile .
```

## Режимы запуска

### Интерактивная оболочка

```powershell
docker run --rm -it rtk2026:latest bash
```

### diff_robot на трассе (Gazebo + SLAM + Nav2 + explorer)

```powershell
.\scripts\run_rtk2026_diff_robot.ps1 -Build -Explore -World track
```

Подробно: [RTK_DIFF_TRACK_SIM.md](RTK_DIFF_TRACK_SIM.md).

### Тесты

```powershell
docker run --rm rtk2026:latest bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 -m pytest /workspace/src/rtk2026_driver/test /workspace/src/rtk2026_base/test -v --tb=short"
```

## Передача устройства (Arduino)

```powershell
docker run --rm -it --device COM3 rtk2026:latest bash
```

Внутри контейнера порт может отображаться как `/dev/ttyS3`; задайте `serial_port` в `rtk2026_driver/config/arduino_bridge.yaml`.

## Скрипты

- `scripts/run_rtk2026_diff_robot.ps1` -- diff_robot на трассе (Gazebo + SLAM + Nav2)
- `scripts/run_docker.ps1` -- интерактивный bash в контейнере
- `scripts/run_tests.ps1` -- запуск тестов
