# Launch interactive lane HSV + IPM calibration tool inside the running container.
#
# Usage:
#   .\scripts\tune_lane.ps1            # simulation mode (yellow+white)
#   .\scripts\tune_lane.ps1 -Mode real # real robot mode (two white lines)
#
# Prerequisites:
#   - The container rtk2026_diff_robot_gazebo must be running (run_rtk2026_diff_robot.ps1 -Autorace)
#   - VcXsrv / Xming must be running for the OpenCV GUI windows
#   - Use -LaneCalibration in run_rtk2026_diff_robot.ps1 so detect_lane accepts
#     parameter changes (calibration mode exposes /detect_lane/set_parameters service)
#
# What it does:
#   1. Copies lane_hsv_tuner.py into the container
#   2. Opens an interactive OpenCV window with trackbars for:
#      - White/Yellow HSV thresholds (W/Y H/L/S lo+hi = 12 sliders)
#      - IPM params (press 'i' to open IPM tuner: height, pitch, y_near/far, x_left/right)
#   3. Shows live preview: IPM bird's-eye | overlay | white mask | yellow mask | combined
#   4. Auto-applies HSV to detect_lane node on every slider move (debounced 300ms)
#   5. Press 's' to save /tmp/lane_tuned.yaml inside container
#   6. Copies lane_tuned.yaml back to docker/patches/lane.yaml automatically
#
# Keys (in the image window):
#   s - save + auto-copy to docker/patches/lane.yaml
#   a - manually apply HSV to detect_lane
#   i - open IPM tuner window / apply IPM params
#   r - reset to defaults
#   q - quit

param(
    [string]$Mode = "sim",           # sim | real
    [string]$Container = "rtk2026_diff_robot_gazebo",
    [string]$OutYaml = "docker/patches/lane.yaml",
    [string]$OutIpmYaml = "docker/patches/ipm_params.yaml"
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $MyInvocation.MyCommand.Path -Parent
$repoRoot = Split-Path $repoRoot -Parent
Set-Location $repoRoot

$tunerSrc  = "scripts/lane_hsv_tuner.py"
$tunerDst  = "/tmp/lane_hsv_tuner.py"

# Verify container is running
$running = docker ps -q -f "name=^/${Container}`$"
if (-not $running) {
    Write-Error "Container '$Container' is not running. Start it first with run_rtk2026_diff_robot.ps1 -Autorace."
    exit 1
}

# DISPLAY for X11
$display = if ($env:DISPLAY) { $env:DISPLAY } else { "host.docker.internal:0.0" }

Write-Host "Copying tuner into container..."
docker cp $tunerSrc "${Container}:${tunerDst}"

Write-Host "Launching lane_hsv_tuner.py (mode=$Mode)..."
Write-Host "Keys: s=save  a=apply HSV  i=IPM tuner  r=reset  q=quit"
Write-Host ""

$cmd = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && DISPLAY=$display python3 $tunerDst --mode $Mode"

docker exec -it $Container bash -c $cmd

# After tuner exits: copy results back if saved
$laneYaml = "/tmp/lane_tuned.yaml"
$ipmYaml  = "/tmp/lane_tuned_ipm.yaml"

$check = docker exec $Container bash -c "test -f $laneYaml && echo yes || echo no"
if ($check.Trim() -eq "yes") {
    Write-Host ""
    Write-Host "Copying $laneYaml → $OutYaml ..."
    docker cp "${Container}:${laneYaml}" $OutYaml
    Write-Host "Saved: $OutYaml"
}

$checkIpm = docker exec $Container bash -c "test -f $ipmYaml && echo yes || echo no"
if ($checkIpm.Trim() -eq "yes") {
    Write-Host "Copying $ipmYaml → $OutIpmYaml ..."
    docker cp "${Container}:${ipmYaml}" $OutIpmYaml
    Write-Host "Saved: $OutIpmYaml"
}

Write-Host ""
Write-Host "Done."
