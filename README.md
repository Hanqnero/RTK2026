# RTK2026

ROS2 Humble workspace для мобильного робота RTK2026 (Raspberry Pi 5, arm64).

Полная документация: [docs/README.md](docs/README.md)

## Быстрый старт (Mac → Pi)

```bash
# 1. Собрать образ для arm64 на Mac:
./scripts/build_pi.sh

# 2. Загрузить на Pi и запустить:
./scripts/deploy_pi.sh

# 3. Открыть Foxglove Studio → ws://192.168.2.2:8765
```

- [Настройка Ethernet Mac ↔ Pi](docs/ETHERNET_MAC_PI_CONNECTION.md)
- [Foxglove Studio](docs/FOXGLOVE_VIEWER.md)
- [Протокол Arduino](docs/protocol_arduino.md)
