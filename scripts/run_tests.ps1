# Run from repo root. Source install/setup.bash (WSL) or use Docker.
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    if (Test-Path "install\setup.bash") {
        wsl bash -c "source install/setup.bash && python3 -m pytest src/rtk2026_driver/test src/rtk2026_base/test -v --tb=short"
    } else {
        Write-Host "Run tests inside Docker: docker run --rm -v ${root}:/ws rtk2026:latest bash -c 'source /ws/install/setup.bash && python3 -m pytest /ws/src/rtk2026_driver/test /ws/src/rtk2026_base/test -v --tb=short'"
        exit 1
    }
} finally {
    Pop-Location
}
