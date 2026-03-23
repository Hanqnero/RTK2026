param(
    [switch]$Build,
    [switch]$Explore,
    [switch]$Teleop,
    [switch]$UseGpu,
    [switch]$Autorace,   # TurtleBot3 Autorace: lane + sign + obstacle (no Nav2)
    [switch]$Map,        # With -Autorace: run slam_toolbox to build map (default: true when -Autorace)
    [switch]$NoMap,      # With -Autorace: do not run SLAM (no /map)
    [switch]$LaneCalibration,   # With -Autorace: run detect_lane in calibration mode (tune in rqt, save lane.yaml)
    [switch]$UseCameraCalibration, # With -Autorace: use turtlebot3_autorace_camera (intrinsic+extrinsic) for /camera/image_projected view
    [switch]$LaneFromCamera,   # With -Autorace: feed detect_lane from /camera/image_projected (requires -UseCameraCalibration)
    [string]$World = "track",  # track | office | city | autorace | <custom-absolute-path>
    [double]$SpawnX = [double]::NaN,
    [double]$SpawnY = [double]::NaN,
    [double]$SpawnZ = [double]::NaN
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $MyInvocation.MyCommand.Path -Parent
$repoRoot = Split-Path $repoRoot -Parent

Set-Location $repoRoot

$image = "rtk2026:latest"

if ($Build) {
    Write-Host "Building image $image..."
    docker build -t $image -f docker/windows/Dockerfile .
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

if (-not $env:ROS_DOMAIN_ID) {
    $env:ROS_DOMAIN_ID = "0"
}

Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"

# DISPLAY for X server on Windows (VcXsrv/Xming)
$display = if ($env:DISPLAY) { $env:DISPLAY } else { "host.docker.internal:0.0" }

$script:autoraceMount = $null
$script:autoraceModelsEnv = $null
$script:robotUrdfPath = $null
# Map World argument to world file path inside the container
# track/office — из пакета diff_robot; city/autorace — из смонтированных репозиториев или образа.
switch ($World.ToLower()) {
    "track"  {
        $worldPath = "/workspace/src/diff_robot/world/silverstone_track.world"
        $spawnArgs = " x:=-5.0 y:=0.0 z:=0.15"
    }
    "office" {
        $worldPath = "/workspace/src/diff_robot/world/office_small.world"
        $spawnArgs = " x:=0.0 y:=0.0 z:=0.15"
    }
    "city"   {
        $worldPath = "/gazebo_worlds/worlds/small_city.world"
        $spawnArgs = " x:=0.0 y:=0.0 z:=0.6"
    }
    # Мир TurtleBot3 Autorace 2020. Приоритет: если на хосте есть turtlebot3_simulations — монтируем и берём мир оттуда; иначе из образа (нужен -Build).
    "autorace" {
        $tb3SimHost = "C:\CursorProject\Robotics\turtlebot3_simulations"
        $worldFile = "turtlebot3_gazebo/worlds/turtlebot3_autorace_2020.world"
        if (Test-Path (Join-Path $tb3SimHost $worldFile)) {
            $worldPath = "/turtlebot3_simulations/" + $worldFile
            $script:autoraceMount = $tb3SimHost
        } else {
            $worldPath = "/workspace/src/turtlebot3_gazebo/worlds/turtlebot3_autorace_2020.world"
            $script:autoraceMount = $null
        }
        $spawnArgs = " x:=0.8 y:=-1.747 z:=0.05"
        # Полный GAZEBO_MODEL_PATH для Gazebo: только каталог с turtlebot3_autorace_2020 (course, знаки). Так модели точно подхватываются.
        $script:autoraceModelsEnv = if ($null -ne $script:autoraceMount) {
            "GAZEBO_MODEL_PATH=/turtlebot3_simulations/turtlebot3_gazebo/models"
        } else {
            "GAZEBO_MODEL_PATH=/workspace/src/turtlebot3_gazebo/models"
        }
        $script:robotUrdfPath = "ROBOT_URDF_PATH=/workspace/src/diff_robot/urdf/diff_robot_fifth.urdf"
    }
    default  {
        $worldPath = $World
        $spawnArgs = ""
    }
}
# Переопределение координат спауна (если робот оказывается в препятствиях)
if (-not [double]::IsNaN($SpawnX)) { $spawnArgs = " x:=$SpawnX y:=" + $(if (-not [double]::IsNaN($SpawnY)) { $SpawnY } else { "0.0" }) + " z:=" + $(if (-not [double]::IsNaN($SpawnZ)) { $SpawnZ } else { "0.05" }) }
elseif (-not [double]::IsNaN($SpawnY)) { $spawnArgs = $spawnArgs -replace " y:=[\d.-]+", " y:=$SpawnY" }
elseif (-not [double]::IsNaN($SpawnZ)) { $spawnArgs = $spawnArgs -replace " z:=[\d.-]+", " z:=$SpawnZ" }

# Для мира autorace: задержки; путь к URDF задаётся через /tmp/robot_urdf_path (см. writeRobotUrdfPath).
$launchExtra = ""
if ($World.ToLower() -eq "autorace") {
    $launchExtra = " spawn_delay:=50.0 nav2_delay:=70.0"
}
# Внутри контейнера: при autorace пишем путь к URDF в /tmp, чтобы launch гарантированно подхватил его (и для spawn, и для robot_description).
$writeRobotUrdfPath = ""
if ($null -ne $script:robotUrdfPath) {
    $writeRobotUrdfPath = "echo /workspace/src/diff_robot/urdf/diff_robot_fifth.urdf > /tmp/robot_urdf_path && "
}
# С -Autorace: запуск пайплайна TurtleBot3 Autorace (полоса, знаки, объезд препятствий) вместо Nav2/SLAM.
$launchPkg = "rtk2026_simulation"
$launchFile = if ($Autorace) { "rtk2026_diff_robot_autorace.launch.py" } else { "rtk2026_diff_robot_track.launch.py" }
$launchArgs = "world:=" + $worldPath + $spawnArgs + $launchExtra + " use_sim_time:=true"
if (-not $Autorace) {
    $launchArgs += " explore:=" + $(if ($Explore) { "true" } else { "false" })
} else {
    # Autorace: по умолчанию строим карту (slam_toolbox) для визуализации в RViz; -NoMap отключает
    $useSlam = -not $NoMap -or $Map
    if ($useSlam) {
        $launchArgs += " use_slam:=true rvizconfig:=/workspace/src/diff_robot/urdf/rviz_autorace.rviz"
    } else {
        $launchArgs += " use_slam:=false"
    }
    if ($LaneCalibration) { $launchArgs += " lane_calibration_mode:=true" }
    if ($UseCameraCalibration) { $launchArgs += " use_camera_calibration:=true" }
    if ($LaneFromCamera) { $launchArgs += " lane_image_source:=camera" }
}
# С -Teleop: launch в фоне, через 50 с запуск teleop_twist_keyboard в этом же терминале (управление без второго окна).
$launchCmd = $writeRobotUrdfPath + "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch " + $launchPkg + " " + $launchFile + " " + $launchArgs
if ($Teleop) {
    $cmd = $launchCmd + " & sleep 50 && ros2 run teleop_twist_keyboard teleop_twist_keyboard; wait"
} else {
    $cmd = $launchCmd + "; wait"
}

$containerName = "rtk2026_diff_robot_gazebo"
if (docker ps -a -q -f "name=^/${containerName}`$") {
    docker rm -f $containerName | Out-Null
}
if ($Autorace) {
    Write-Host "Starting RTK2026 diff_robot + TurtleBot3 Autorace (lane, sign, obstacle; no Nav2)..."
} else {
    Write-Host "Starting RTK2026 diff_robot Gazebo + RViz + Nav2 (with SLAM)..."
}
if ($Teleop) {
    Write-Host "Teleop: in ~50s this terminal will show keyboard control (no second window needed)."
}
if ($Autorace) {
    $useSlamMsg = -not $NoMap -or $Map
    if ($useSlamMsg) {
        Write-Host "Autorace + Map: lane/sign/obstacle and slam_toolbox (map builds as robot moves; map is for visualization only, not Nav2)."
    } else {
        Write-Host "Autorace (no map): lane/sign/obstacle will drive /cmd_vel. Use -Map or omit -NoMap to build map in RViz."
    }
    if ($LaneCalibration) { Write-Host "Lane calibration mode: detect_lane is_detection_calibration_mode=true. Tune in rqt, then save lane.yaml (see docs/AUTONOMOUS_DRIVING.md)." }
    if ($UseCameraCalibration) { Write-Host "Camera calibration: turtlebot3_autorace_camera (intrinsic+extrinsic) for image_projected; IPM relay disabled." }
} elseif ($Explore) {
    Write-Host "Autonomous exploration enabled: robot will build the map by itself (frontier exploration)."
} else {
    Write-Host "In RViz: set 2D Pose Estimate once, then use 2D Nav Goal for navigation."
}

# С -Teleop нужен интерактивный TTY, чтобы teleop_twist_keyboard читал клавиатуру
$gazeboWorldsHost = "C:\CursorProject\Robotics\gazebo_models_worlds_collection"
$dockerRunArgs = @(
    "run", "--rm",
    "-e", "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID",
    "-e", "DISPLAY=$display",
    "-v", "${gazeboWorldsHost}:/gazebo_worlds:ro",
    "--name", $containerName
)
if ($null -ne $script:autoraceMount) {
    $dockerRunArgs += "-v"; $dockerRunArgs += "${script:autoraceMount}:/turtlebot3_simulations:ro"
    Write-Host "Autorace: using world from host turtlebot3_simulations."
}
if ($null -ne $script:autoraceModelsEnv) {
    $dockerRunArgs += "-e"; $dockerRunArgs += $script:autoraceModelsEnv
    $dockerRunArgs += "-e"; $dockerRunArgs += "GAZEBO_MODEL_DATABASE_URI="
}
if ($null -ne $script:robotUrdfPath) {
    $dockerRunArgs += "-e"; $dockerRunArgs += $script:robotUrdfPath
    Write-Host "Autorace: using scaled robot (diff_robot_fifth.urdf, 1/10 size, fits road and buildings)."
}
if ($UseGpu) {
    $dockerRunArgs += "--gpus"; $dockerRunArgs += "all"
    $dockerRunArgs += "-e"; $dockerRunArgs += "__GLX_VENDOR_LIBRARY_NAME=nvidia"
    $dockerRunArgs += "-e"; $dockerRunArgs += "__NV_PRIME_RENDER_OFFLOAD=1"
    $dockerRunArgs += "-e"; $dockerRunArgs += "LIBGL_ALWAYS_SOFTWARE=0"
    Write-Host "GPU: --gpus all + OpenGL env (WSL2 + nvidia-container-toolkit required)."
}
if ($Teleop) { $dockerRunArgs += "-it" }
& docker @dockerRunArgs $image bash -c $cmd

