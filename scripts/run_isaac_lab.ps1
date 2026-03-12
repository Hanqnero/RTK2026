# Launch Isaac Lab with RTK2026 scene (ground, light, robot, optional ROS2 /clock).
# Run from RTK2026 repo root: .\scripts\run_isaac_lab.ps1 or .\scripts\run_isaac_lab.bat
# Expects Isaac Lab as sibling of RTK folder (e.g. workspace/IsaacLab, workspace/RTK/RTK2026). Override with -IsaacLabPath.

param([string]$IsaacLabPath = "")

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
if (-not (Test-Path $repoRoot)) {
    Write-Host "Error: Script must be run from RTK2026 repo or with full path to this .ps1 file."
    exit 1
}
$sceneScript = Join-Path $repoRoot "isaac_lab\run_rtk2026_scene.py"
if ($IsaacLabPath) {
    $isaacLabRoot = $IsaacLabPath
} else {
    $workspaceRoot = Split-Path (Split-Path $repoRoot -Parent) -Parent
    $isaacLabRoot = Join-Path $workspaceRoot "IsaacLab"
}

if (-not (Test-Path $sceneScript)) {
    Write-Host "Scene script not found: $sceneScript"
    exit 1
}

if (-not (Test-Path $isaacLabRoot)) {
    Write-Host "Isaac Lab root not found: $isaacLabRoot. Expected next to RTK folder or pass -IsaacLabPath."
    exit 1
}

$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
Write-Host "Starting Isaac Lab with RTK2026 scene. Press Play in the simulator."
Write-Host "Scene: ground, light, diff-drive robot. Press Play in the simulator."
Write-Host "ROS2: Window > Extensions > isaacsim.ros2.bridge (see docs/ROS2_ISAAC_WINDOWS.md). Add --no_robot for minimal (no robot)."
Set-Location $isaacLabRoot
& .\isaaclab.bat -p $sceneScript --num_envs 1
