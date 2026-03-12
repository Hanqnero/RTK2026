# Stop all running containers (optionally only rtk2026 image). Run from any dir.
# Usage: .\scripts\stop_docker_containers.ps1
#        .\scripts\stop_docker_containers.ps1 -All  (stop every running container)

param(
    [switch]$All
)

$repoRoot = Split-Path $PSScriptRoot -Parent
Set-Location $repoRoot

if ($All) {
    $ids = docker ps -q
} else {
    $ids = docker ps -q --filter "ancestor=rtk2026:latest"
    if (-not $ids) {
        $ids = docker ps -q --filter "name=rtk2026"
    }
}
if ($ids) {
    $ids | ForEach-Object { docker stop $_ }
    Write-Host "Stopped container(s)."
} else {
    Write-Host "No matching running containers (rtk2026). Use -All to stop all."
}
