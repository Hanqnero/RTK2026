# Build RTK2026 image (if needed) and run interactive bash in container.
# From repo root: .\scripts\run_docker.ps1
# Optional: .\scripts\run_docker.ps1 -Build  (force rebuild before run)

param([switch]$Build)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
if (-not (Test-Path (Join-Path $repoRoot "docker\windows\Dockerfile"))) {
    Write-Host "Repo root with docker/ not found. Run from RTK2026 root."
    exit 1
}

Set-Location $repoRoot

if ($Build) {
    Write-Host "Building image..."
    docker compose -f docker/docker-compose.yml build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "0" }
Write-Host "Starting container (ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID). Inside: source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash"
docker run --rm -it -e ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID rtk2026:latest bash
