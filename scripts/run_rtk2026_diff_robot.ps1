param(
    [switch]$Build,
    [switch]$Explore,
    [string]$World = "track"  # track | office | city | <custom-absolute-path-inside-container>
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $MyInvocation.MyCommand.Path -Parent
$repoRoot = Split-Path $repoRoot -Parent

Set-Location $repoRoot

$image = "rtk2026:latest"

if ($Build) {
    Write-Host "Building image $image..."
    docker build -t $image -f docker/Dockerfile.windows .
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

if (-not $env:ROS_DOMAIN_ID) {
    $env:ROS_DOMAIN_ID = "0"
}

Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"

# DISPLAY for X server on Windows (VcXsrv/Xming)
$display = if ($env:DISPLAY) { $env:DISPLAY } else { "host.docker.internal:0.0" }

# Map World argument to world file path inside the container
# track/office берём из пакета робота, city — ИЗ СКЛОНИРОВАННОЙ РЕПЫ, смонтированной в контейнер.
# Одновременно задаём удобные стартовые координаты для разных миров, чтобы не спауниться в стене.
switch ($World.ToLower()) {
    "track"  {
        $worldPath = "/workspace/src/diff_robot/world/silverstone_track.world"
        # Чуть левее/позади центра, над дорогой
        $spawnArgs = " x:=-5.0 y:=0.0 z:=0.15"
    }
    "office" {
        $worldPath = "/workspace/src/diff_robot/world/office_small.world"
        $spawnArgs = " x:=0.0 y:=0.0 z:=0.15"
    }
    # small_city.world напрямую из gazebo_models_worlds_collection на хосте
    "city"   {
        $worldPath = "/gazebo_worlds/worlds/small_city.world"
        # В городе поднимем робот повыше, чтобы гарантированно не утонул в рельефе
        $spawnArgs = " x:=0.0 y:=0.0 z:=0.6"
    }
    default  {
        $worldPath = $World
        $spawnArgs = ""
    }
}

# Внутри контейнера используем единый RTK‑launch:
# rtk2026_simulation/rtk2026_diff_robot_track.launch.py
if ($Explore) {
    $cmd = "source /opt/ros/humble/setup.bash && " +
           "source /workspace/install/setup.bash && " +
           "ros2 launch rtk2026_simulation rtk2026_diff_robot_track.launch.py world:=" + $worldPath + $spawnArgs + " use_sim_time:=true explore:=true; wait"
} else {
    $cmd = "source /opt/ros/humble/setup.bash && " +
           "source /workspace/install/setup.bash && " +
           "ros2 launch rtk2026_simulation rtk2026_diff_robot_track.launch.py world:=" + $worldPath + $spawnArgs + " use_sim_time:=true explore:=false; wait"
}

$containerName = "rtk2026_diff_robot_gazebo"
if (docker ps -a -q -f "name=^/${containerName}`$") {
    docker rm -f $containerName | Out-Null
}
Write-Host "Starting RTK2026 diff_robot Gazebo + RViz + Nav2 (with SLAM)..."
if ($Explore) {
    Write-Host "Autonomous exploration enabled: robot will build the map by itself (frontier exploration)."
} else {
    Write-Host "In RViz: set 2D Pose Estimate once, then use 2D Nav Goal for navigation."
}

# Монтируем склонированную коллекцию миров внутрь контейнера по пути /gazebo_worlds
$gazeboWorldsHost = "C:\CursorProject\Robotics\gazebo_models_worlds_collection"
docker run --rm `
    -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID `
    -e DISPLAY=$display `
    -v "${gazeboWorldsHost}:/gazebo_worlds:ro" `
    --name $containerName `
    $image bash -c "$cmd"

