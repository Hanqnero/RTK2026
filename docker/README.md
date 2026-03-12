# Docker

Build from repository root:

```bash
docker build -f docker/Dockerfile -t rtk2026:latest .
```

Or with compose:

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up
```

Default command runs: `ros2 launch rtk2026_bringup rtk2026.launch.py`
