# Run RTK2026 ROS2 stack for Isaac Sim test.
# 1) Start Isaac Sim with a scene that has ROS2 bridge (/clock, /odom, /cmd_vel), then press Play.
# 2) Run this script (from repo root, after: source install/setup.bash or equivalent on Windows).
# 3) In another terminal: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

$ErrorActionPreference = "Stop"
if (-not $env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID = "0" }
Write-Host "ROS_DOMAIN_ID=$env:ROS_DOMAIN_ID. Start Isaac Sim first and press Play, then run our launch."
$launch = "rtk2026_simulation"
$pkg = "simulation"
$args = "use_sim_time:=true use_fake_scan:=true use_slam:=true"
Write-Host "Running: ros2 launch $launch $pkg.launch.py $args"
ros2 launch $launch $pkg.launch.py $args
