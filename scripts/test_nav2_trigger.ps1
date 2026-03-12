# Step-by-step check of trigger_nav2_bringup and Nav2 launch logic.
# Run from RTK2026 root. Use -Build to rebuild image first.
# 1) List rtk2026_peripherals executables (must include trigger_nav2_bringup).
# 2) Run trigger_nav2_bringup with delay_sec:=1; expect "Waiting 1ds" then service unavailable (exit 1).

param(
    [switch]$Build
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
if (-not (Test-Path (Join-Path $repoRoot "docker\Dockerfile"))) {
    Write-Host "Repo root with docker/Dockerfile not found. Run from RTK2026 root."
    exit 1
}

Set-Location $repoRoot

if ($Build) {
    Write-Host "Step 0: Building image..."
    docker compose -f docker/docker-compose.yml build --no-cache
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Host ""
}

$src = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash"

Write-Host "Step 1: List rtk2026_peripherals executables (expect trigger_nav2_bringup)..."
$out = docker run --rm rtk2026:latest bash -c "$src && ros2 pkg executables rtk2026_peripherals" 2>&1 | Out-String
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
if ($out -notmatch "trigger_nav2_bringup") {
    Write-Host "FAIL: trigger_nav2_bringup not in list. Rebuild: .\scripts\test_nav2_trigger.ps1 -Build"
    Write-Host "Output: $out"
    exit 1
}
Write-Host $out.Trim()
Write-Host ""

Write-Host "Step 2: Run trigger_nav2_bringup with delay_sec:=1 (expect 'Waiting 1s' then STARTUP failed / not available)..."
$cmd = "$src && ros2 run rtk2026_peripherals trigger_nav2_bringup --ros-args -p delay_sec:=1"
$prevErr = $ErrorActionPreference
$ErrorActionPreference = "Continue"
$run = docker run --rm rtk2026:latest bash -c $cmd 2>&1 | Out-String
$exitCode = $LASTEXITCODE
$ErrorActionPreference = $prevErr
Write-Host $run.Trim()
if ($run -notmatch "Waiting 1s") {
    Write-Host "FAIL: Expected log 'Waiting 1s before...' (node or param issue)"
    exit 1
}
if ($run -notmatch "not available|STARTUP failed") {
    Write-Host "WARN: Expected 'not available' or 'STARTUP failed' (no lifecycle_manager); exit code $exitCode"
}
Write-Host ""
Write-Host "Steps 1-2 OK. Full pipeline: run_docker_slam.ps1 then send_nav_goal.ps1"
