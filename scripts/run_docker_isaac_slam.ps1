# Run ROS2 stack in Docker; can be used with or without Isaac Sim.
# Without Isaac: use_fake_odom + clock_publisher (run_docker_slam.ps1 or this script).
# With Isaac: start Isaac first, then run this script with use_fake_odom:=false when Isaac publishes /odom.
# This script uses use_fake_odom:=true by default (Docker-only, no Isaac). RViz: run on host if needed.
# Optional: -Build to rebuild image. -NoNavigation to skip Nav2.

param(
    [switch]$Build,
    [switch]$NoNavigation
)

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
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
Write-Host "Start Isaac Sim (or Isaac Lab) with ROS2 bridge first, load RTK2026 scene, press Play."
Write-Host "Launch in Docker: robot_state_publisher, odom_tf_broadcaster, fake_scan, slam_toolbox, nav2 (no RViz - no display in container)."

$nav = if ($NoNavigation) { "false" } else { "true" }
# use_fake_odom:=true publishes static tf odom->base_link so Nav2 starts when Isaac does not publish /odom (e.g. ROS2 bridge disabled)
$cmd = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_fake_odom:=true use_slam:=true use_navigation:=$nav use_rviz:=false"

docker run --rm -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID rtk2026:latest bash -c $cmd
