# One container: base + SLAM + Nav2 (autostart:=false + trigger_nav2_bringup after 60s).
# Use when two-container DDS does not see TF (e.g. bridge network). Run from RTK2026 root.
# Optional: -Build to rebuild image.

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
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID One container (base + Nav2 with trigger). Wait ~90s for Managed nodes are active."

$cmd = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_fake_odom:=true use_slam:=true use_navigation:=true use_rviz:=false"
docker run --rm -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID --name rtk2026_one rtk2026:latest bash -c $cmd
