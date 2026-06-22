param(
    [switch]$NoBuild,
    [switch]$NoGui,
    [switch]$KeepRunning
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
$composeFile = Join-Path $repoRoot "docker\docker-compose.sim.yml"

if (-not (Test-Path $composeFile)) {
    throw "Run this script from a complete RTK2026 checkout."
}

docker info *> $null
if ($LASTEXITCODE -ne 0) {
    throw "Docker Desktop is not running."
}

$services = @("gazebo", "robot", "odometry", "mearm_driver")
$composeArgs = @("compose", "-f", $composeFile)
if (-not $NoGui) {
    $composeArgs += @("--profile", "debug-ui")
    $services += @("gazebo_gui", "rviz_novnc")
}

$upArgs = $composeArgs + @("up", "-d")
if (-not $NoBuild) {
    $upArgs += "--build"
}
$upArgs += $services

Write-Host "Starting RTK2026 simulation with MeArm..."
& docker @upArgs
if ($LASTEXITCODE -ne 0) {
    throw "Unable to start the simulation stack."
}

if (-not $NoGui) {
    Write-Host "Gazebo: http://localhost:6080/vnc.html"
    Write-Host "RViz:   http://localhost:6082/vnc.html"
}
Write-Host "Base: I/K forward/back, J/L turn. Arm: Q/A, W/S, E/D, R/F."
Write-Host "Space stops the base and centers the arm. X or Esc exits."

$teleopCommand = @"
source /opt/ros/`${ROS_DISTRO:-jazzy}/setup.bash
source /workspace/install/setup.bash
ros2 run mearm_driver mearm_manual_controller --ros-args -p control_mode:=keyboard
"@

try {
    $runArgs = $composeArgs + @(
        "run", "--rm", "--no-deps",
        "robot", "bash", "-lc", $teleopCommand
    )
    & docker @runArgs
}
finally {
    if (-not $KeepRunning) {
        Write-Host "Stopping the simulation stack..."
        $downArgs = $composeArgs + @("down")
        & docker @downArgs
    }
}
