# Run base stack only (like zero-to-slam: one container = one concern).
# Base: robot_state_publisher, fake_scan, clock_publisher, static odom/map/tf, slam_toolbox. No Nav2.
# Run Nav2 in a second container: .\scripts\run_docker_slam_nav2.ps1 (in another terminal).
# Run from RTK2026 root. Optional: -Build to rebuild image, -Rviz for RViz (need DISPLAY).

param(
    [switch]$Build,
    [switch]$Rviz,
    [switch]$Detach
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

$net = "rtk2026_net"
if (-not (docker network ls -q -f "name=^${net}$")) { docker network create $net }
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
Write-Host "Base only (no Nav2). Start Nav2 in another terminal: .\scripts\run_docker_slam_nav2.ps1"

$rvizFlag = $Rviz.IsPresent.ToString().ToLower()
$cmd = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_fake_odom:=true use_slam:=true use_navigation:=false use_rviz:=$rvizFlag"
$cycloneXml = '<CycloneDDS><Domain><Discovery><Peers><Peer address="rtk2026_nav2"/></Peers></Discovery></Domain></CycloneDDS>'
$cycloneDir = Join-Path $env:TEMP "rtk2026_cyclone"
$null = New-Item -ItemType Directory -Force -Path $cycloneDir
$cycloneFile = Join-Path $cycloneDir "cyclone_base.xml"
$utf8NoBom = New-Object System.Text.UTF8Encoding $false
[System.IO.File]::WriteAllText($cycloneFile, $cycloneXml, $utf8NoBom)
$cycloneUri = "file:///tmp/cyclone_base.xml"
$runArgs = @("run", "--rm", "-e", "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID", "-e", "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", "-e", "CYCLONEDDS_URI=$cycloneUri", "-v", "${cycloneFile}:/tmp/cyclone_base.xml:ro", "--network", $net, "--name", "rtk2026_base", "rtk2026:latest", "bash", "-c", $cmd)
if ($Rviz) {
    $display = if ($env:DISPLAY) { $env:DISPLAY } else { "host.docker.internal:0" }
    $runArgs = @("run", "--rm", "-e", "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID", "-e", "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", "-e", "CYCLONEDDS_URI=$cycloneUri", "-v", "${cycloneFile}:/tmp/cyclone_base.xml:ro", "-e", "DISPLAY=$display", "--network", $net, "--name", "rtk2026_base", "rtk2026:latest", "bash", "-c", $cmd)
}
if ($Detach) {
    $runArgs = @("run", "-d") + $runArgs[1..($runArgs.Length-1)]
    # omit --rm when detaching so container remains for logs if it exits
    $runArgs = $runArgs | Where-Object { $_ -ne "--rm" }
}
docker @runArgs
