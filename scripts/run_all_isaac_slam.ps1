# Start Isaac Lab (RTK2026 scene) AND Docker ROS2 stack. Use only when you need the simulator.
# For Docker-only (fake_scan, SLAM, Nav2): use run_docker_slam.ps1 instead.
# Run from RTK2026 root: .\scripts\run_all_isaac_slam.ps1
# Isaac Lab runs in background; Docker runs in this terminal (Ctrl+C stops Docker only).

param(
    [switch]$Build,
    [switch]$NoNavigation,
    [int]$DelaySeconds = 15
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
Set-Location $repoRoot

$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"

# 1) Start Isaac Lab in background
Write-Host "Starting Isaac Lab with RTK2026 scene (background)..."
$isaacJob = Start-Process -FilePath "powershell" -ArgumentList "-ExecutionPolicy Bypass -File `"$repoRoot\scripts\run_isaac_lab.ps1`"" -WorkingDirectory $repoRoot -PassThru -WindowStyle Normal
Write-Host "Isaac Lab PID: $($isaacJob.Id). Press Play in the simulator when the window opens."

# 2) Wait for Isaac to open
Write-Host "Waiting $DelaySeconds s for Isaac Lab to open..."
Start-Sleep -Seconds $DelaySeconds

# 3) Run Docker stack (foreground)
Write-Host "Starting Docker ROS2 stack (SLAM + Nav2)..."
& "$repoRoot\scripts\run_docker_isaac_slam.ps1" @PSBoundParameters
