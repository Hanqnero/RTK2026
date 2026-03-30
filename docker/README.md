# Docker

## Raspberry Pi (arm64)

Сборка на Mac (Apple Silicon):

```bash
./scripts/build_pi.sh
# или вручную:
docker buildx build --platform linux/arm64 -t rtk2026:latest -f docker/pi/Dockerfile .
```

Загрузка на Pi и запуск:

```bash
./scripts/deploy_pi.sh
# или вручную:
docker save rtk2026:latest | gzip | ssh pi@192.168.2.2 "gunzip | docker load"
ssh pi@192.168.2.2 "docker run -d --name rtk2026 --privileged -v /dev:/dev --network host rtk2026:latest"
```

## Windows (симуляция, amd64)

```powershell
.\scripts\run_rtk2026_diff_robot.ps1 -Build -Explore -World track
```
