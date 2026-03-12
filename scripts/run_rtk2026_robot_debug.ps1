param(
    [switch]$Build,
    [switch]$Explore
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $MyInvocation.MyCommand.Path -Parent
$repoRoot = Split-Path $repoRoot -Parent

Set-Location $repoRoot

$image = "rtk2026:latest"

if ($Build) {
    Write-Host "Building image $image..."
    docker build -t $image -f docker/Dockerfile .
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

if (-not $env:ROS_DOMAIN_ID) {
    $env:ROS_DOMAIN_ID = "0"
}

Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"

# DISPLAY для X‑сервера на Windows (VcXsrv/Xming)
$display = if ($env:DISPLAY) { $env:DISPLAY } else { "host.docker.internal:0.0" }

# В этом сценарии:
# - реальный робот (Raspberry Pi) поднимает драйвер/базу/лидар, публикует /odom и /scan;
# - контейнер на ПК поднимает SLAM + Nav2 + explorer + RViz + teleop.
#
# Предполагается, что:
# - ROS_DOMAIN_ID на роботе и в контейнере совпадают;
# - DDS‑трафик между роботом и контейнером проходит по сети (CycloneDDS или FastDDS, см. доки ROS2).

$nav2Cmd = "ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=false use_fake_scan:=false use_fake_odom:=false use_slam:=true use_navigation:=true use_localization:=true use_rviz:=true rviz_config:=rtk2026_sim_slam.rviz"

if ($Explore) {
    $exploreCmd = "ros2 launch rtk2026_nav2_explorer rtk2026_explorer.launch.py use_sim_time:=false"
} else {
    $exploreCmd = ""
}

# Teleop для управления роботом с клавиатуры (cmd_vel на /cmd_vel)
$teleopCmd = "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

$cmd = "source /opt/ros/humble/setup.bash && " +
       "source /workspace/install/setup.bash && " +
       $nav2Cmd + " & " +
       "sleep 20 && " +
       ($exploreCmd -ne "" ? ($exploreCmd + " & ") : "") +
       $teleopCmd + "; wait"

$containerName = "rtk2026_robot_debug"
if (docker ps -a -q -f "name=^/${containerName}`$") {
    docker rm -f $containerName | Out-Null
}

Write-Host "Starting RTK2026 robot debug container (SLAM + Nav2 + RViz + teleop)..."
if ($Explore) {
    Write-Host "Explorer is enabled: frontier-based autonomous exploration will run on real robot's map."
}

docker run --rm `
    -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID `
    -e DISPLAY=$display `
    --name $containerName `
    $image bash -c "$cmd"

