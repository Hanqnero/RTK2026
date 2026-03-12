# Send a test Nav2 goal. With split containers use the Nav2 container (run_docker_slam_nav2.ps1).
# Run from RTK2026 root: .\scripts\send_nav_goal.ps1. Optional: -X, -Y, -Yaw (degrees).

param(
    [double]$X = 1.0,
    [double]$Y = 0.0,
    [double]$Yaw = 0.0
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path $PSScriptRoot -Parent
Set-Location $repoRoot

$id = (docker ps --filter "name=rtk2026_nav2" --format "{{.ID}}" | Select-Object -First 1)
if (-not $id) { $id = (docker ps --filter "ancestor=rtk2026:latest" --format "{{.ID}}" | Select-Object -First 1) }
if (-not $id) {
    Write-Host "No running container. Start base: .\scripts\run_docker_slam.ps1; then Nav2: .\scripts\run_docker_slam_nav2.ps1"
    exit 1
}

$yawRad = $Yaw * 3.14159265359 / 180.0
$qz = [math]::Sin($yawRad / 2.0)
$qw = [math]::Cos($yawRad / 2.0)

$yaml = @"
pose:
  header:
    frame_id: map
  pose:
    position: { x: $X, y: $Y, z: 0.0 }
    orientation: { x: 0.0, y: 0.0, z: $qz, w: $qw }
"@
$tmpFile = [System.IO.Path]::GetTempFileName()
$yaml | Set-Content -Path $tmpFile -Encoding UTF8

try {
    docker cp $tmpFile "${id}:/tmp/nav_goal.yaml"
    $sendScript = @'
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
goal=$(cat /tmp/nav_goal.yaml)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$goal"
'@
    ($sendScript -replace "`r`n", "`n") | Set-Content -Path "$repoRoot\tmp_send_goal.sh" -Encoding ASCII -NoNewline
    docker cp "$repoRoot\tmp_send_goal.sh" "${id}:/tmp/send_goal.sh"
    docker exec $id bash /tmp/send_goal.sh
} finally {
    Remove-Item -Path $tmpFile -ErrorAction SilentlyContinue
    Remove-Item -Path "$repoRoot\tmp_send_goal.sh" -ErrorAction SilentlyContinue
}
