param(
    [switch]$BuildImage,
    [switch]$NoNavigation,
    [string]$IsaacLabPath = ""
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
if (-not (Test-Path (Join-Path $repoRoot "docker\\Dockerfile"))) {
    Write-Host "Repo root with docker/Dockerfile not found. Run from RTK2026 root."
    exit 1
}

Set-Location $repoRoot

# 1. Build Docker image (optional)
if ($BuildImage) {
    Write-Host "Building Docker image rtk2026:latest..."
    docker compose -f docker/docker-compose.yml build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# 2. Start Isaac Lab with RTK2026 scene in a separate PowerShell process
$runIsaacScript = Join-Path $repoRoot "scripts\\run_isaac_lab.ps1"
if (-not (Test-Path $runIsaacScript)) {
    Write-Host "run_isaac_lab.ps1 not found at $runIsaacScript"
    exit 1
}

Write-Host "Starting Isaac Lab with RTK2026 scene in a separate window..."
$isaacArgs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", "`"$runIsaacScript`"")
if ($IsaacLabPath) {
    $isaacArgs += @("-IsaacLabPath", "`"$IsaacLabPath`"")
}
Start-Process powershell.exe -ArgumentList $isaacArgs -WorkingDirectory $repoRoot | Out-Null

Write-Host ""
Write-Host "Wait for Isaac Lab to open, load the RTK2026 scene and press Play."
Write-Host "ROS2 bridge must be enabled inside Isaac (isaacsim.ros2.bridge)."
Write-Host ""

# 3. Give Isaac some time to start before launching Docker stack
Start-Sleep -Seconds 20

# 4. Launch ROS2 stack in Docker (Isaac SLAM + Nav2) with same ROS_DOMAIN_ID
$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID"
Write-Host "Starting Docker ROS2 stack (slam_toolbox + Nav2)..."

$runDockerScript = Join-Path $repoRoot "scripts\\run_docker_isaac_slam.ps1"
if (-not (Test-Path $runDockerScript)) {
    Write-Host "run_docker_isaac_slam.ps1 not found at $runDockerScript"
    exit 1
}

# Pass -NoNavigation and -Build flags through as appropriate
& $runDockerScript @(
    $(if ($BuildImage) { "-Build" }),
    $(if ($NoNavigation) { "-NoNavigation" })
) | Write-Host

