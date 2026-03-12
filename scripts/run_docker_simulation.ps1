# Run simulation launch in Docker (for Isaac Lab test).
# 1) Start Isaac Sim/Lab with ROS2 bridge, press Play.
# 2) Run: .\scripts\run_docker_simulation.ps1
# Optional: -Build to rebuild image first.

param([switch]$Build)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
if (-not (Test-Path (Join-Path $repoRoot "docker\Dockerfile"))) {
    Write-Host "Repo root with docker/Dockerfile not found. Run from RTK2026 root."
    exit 1
}

Set-Location $repoRoot

if ($Build) {
    Write-Host "Building image..."
    docker compose -f docker/docker-compose.yml build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID. Start Isaac Sim first and press Play, then this launch runs in container."
$cmd = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_slam:=true"
docker run --rm -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID rtk2026:latest bash -c $cmd
