# Launch WheeledLab Visual scene (MuSHR + camera + traversability / semantic navigation) via Isaac Lab.
# Expects Isaac Lab and WheeledLab as siblings of RTK folder (e.g. workspace/IsaacLab, workspace/WheeledLab). Override with -IsaacLabPath / -WheeledLabPath.
# Run from RTK2026 root: .\scripts\run_wheeledlab_visual.ps1 or .\scripts\run_wheeledlab_visual.bat

param([string]$IsaacLabPath = "", [string]$WheeledLabPath = "")

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
$scriptPath = Join-Path $repoRoot "isaac_lab\run_wheeledlab_visual_scene.py"
$workspaceRoot = Split-Path (Split-Path $repoRoot -Parent) -Parent
if ($IsaacLabPath) { $isaacLabRoot = $IsaacLabPath } else { $isaacLabRoot = Join-Path $workspaceRoot "IsaacLab" }
if ($WheeledLabPath) { $wheeledLabRoot = $WheeledLabPath } else { $wheeledLabRoot = Join-Path $workspaceRoot "WheeledLab" }

if (-not (Test-Path $scriptPath)) {
    Write-Host "Script not found: $scriptPath"
    exit 1
}
if (-not (Test-Path $isaacLabRoot)) {
    Write-Host "Isaac Lab not found: $isaacLabRoot. Expected next to RTK folder or pass -IsaacLabPath."
    exit 1
}
if (-not (Test-Path $wheeledLabRoot)) {
    Write-Host "WheeledLab not found: $wheeledLabRoot. Clone from https://github.com/UWRobotLearning/WheeledLab or pass -WheeledLabPath."
    exit 1
}

Write-Host "Starting WheeledLab Visual scene (MuSHR + camera + traversability). Press Play in the simulator."
Set-Location $isaacLabRoot
& .\isaaclab.bat -p $scriptPath --num_envs 1
