param(
    [switch]$Build,
    [switch]$Explore,
    [string]$World = "city"  # city | track | <absolute-path-inside-container>
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

# DISPLAY for X server on Windows (VcXsrv/Xming)
$display = if ($env:DISPLAY) { $env:DISPLAY } else { "host.docker.internal:0.0" }

# Map World argument to world file path inside the container
# city и track — ИЗ СКЛОНИРОВАННОЙ РЕПЫ gazebo_models_worlds_collection, смонтированной в контейнер.
switch ($World.ToLower()) {
    "city" {
        $worldPath = "/gazebo_worlds/worlds/small_city.world"
        $spawnArgs = " x:=0.0 y:=0.0 z:=0.6"
    }
    "track" {
        $worldPath = "/gazebo_worlds/worlds/silverstone_track.world"
        $spawnArgs = " x:=-5.0 y:=0.0 z:=0.15"
    }
    default {
        $worldPath = $World
        $spawnArgs = ""
    }
}

$simLaunch = "ros2 launch rtk2026_bringup rtk2026_sim_slam_explore.launch.py world:=" + $worldPath + $spawnArgs

if ($Explore) {
    # Полный автономный пайплайн: Gazebo + SLAM + Nav2 + frontier explorer.
    $cmd = "source /opt/ros/humble/setup.bash && " +
           "source /workspace/install/setup.bash && " +
           $simLaunch + "; wait"
} else {
    # Только Gazebo + SLAM + Nav2 без explorer: можно ставить цели руками из RViz.
    $cmd = "source /opt/ros/humble/setup.bash && " +
           "source /workspace/install/setup.bash && " +
           $simLaunch + "; wait"
}

$containerName = "rtk2026_gazebo"
if (docker ps -a -q -f "name=^/${containerName}`$") {
    docker rm -f $containerName | Out-Null
}

Write-Host "Starting RTK2026 Gazebo + SLAM + Nav2 (world=$World)..."
if ($Explore) {
    Write-Host "Autonomous frontier exploration enabled: robot will build the map by itself."
} else {
    Write-Host "You can send Nav2 goals manually from RViz (when attached)."
}

# External Gazebo worlds (both city and track) from gazebo_models_worlds_collection
$gazeboWorldsHost = "C:\CursorProject\Robotics\gazebo_models_worlds_collection"

docker run --rm `
    -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID `
    -e DISPLAY=$display `
    -v "${gazeboWorldsHost}:/gazebo_worlds:ro" `
    --name $containerName `
    $image bash -c "$cmd"

