# Run only Nav2 in a separate container. Base must be running first: run_docker_slam.ps1.
# Run from RTK2026 root. On Linux use run_docker_slam_nav2.sh (--network host).

param([switch]$Build)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
Set-Location $repoRoot

if ($Build) {
    docker compose -f docker/docker-compose.yml build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

$net = "rtk2026_net"
$baseName = "rtk2026_base"
$baseId = docker ps -q --filter "name=$baseName" | Select-Object -First 1
if (-not $baseId) {
    Write-Host "Container $baseName not running. Start it first: .\scripts\run_docker_slam.ps1"
    exit 1
}

$baseIp = docker inspect -f "{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}" $baseId
if (-not $baseIp) {
    Write-Host "Could not get IP of $baseName. Is it on network $net?"
    exit 1
}

$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
$cycloneXml = "<CycloneDDS><Domain><Discovery><Peers><Peer address=`"$baseIp`"/></Peers></Discovery></Domain></CycloneDDS>"
$cycloneDir = Join-Path $env:TEMP "rtk2026_cyclone"
$null = New-Item -ItemType Directory -Force -Path $cycloneDir
$cycloneFile = Join-Path $cycloneDir "cyclone_nav2.xml"
$utf8NoBom = New-Object System.Text.UTF8Encoding $false
[System.IO.File]::WriteAllText($cycloneFile, $cycloneXml, $utf8NoBom)
$cycloneUri = "file:///tmp/cyclone_nav2.xml"
$paramsPath = "/workspace/install/rtk2026_navigation/share/rtk2026_navigation/config/nav2_params.yaml"
$cmd = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=$paramsPath"

Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID Base IP=$baseIp"
docker run --rm -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e "CYCLONEDDS_URI=$cycloneUri" -v "${cycloneFile}:/tmp/cyclone_nav2.xml:ro" --network $net --name rtk2026_nav2 rtk2026:latest bash -c $cmd
