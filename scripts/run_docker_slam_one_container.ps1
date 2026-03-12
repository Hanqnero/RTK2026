# One container: base (background) then Nav2. Use when two-container DDS discovery fails (e.g. Docker bridge).
# Proves lifecycle bringup works; TF/topics shared in same process space.
# Run from RTK2026 root. Optional: -Build, -Rviz (base only).

param(
    [switch]$Build,
    [switch]$Rviz
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
Set-Location $repoRoot

if ($Build) {
    docker compose -f docker/docker-compose.yml build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
$rvizFlag = $Rviz.IsPresent.ToString().ToLower()
$paramsPath = "/workspace/install/rtk2026_navigation/share/rtk2026_navigation/config/nav2_params.yaml"

$baseCmd = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_fake_odom:=true use_slam:=true use_navigation:=false use_rviz:=$rvizFlag"
$nav2Cmd = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=$paramsPath"

$script = "($baseCmd &); sleep 20; $nav2Cmd"
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
Write-Host "One container: base in background, Nav2 after 20s (wait for 'Managed nodes are active')."
docker run --rm -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID rtk2026:latest bash -c $script
