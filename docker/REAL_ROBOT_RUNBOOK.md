# Real Robot Runbook

This file is the operator runbook for the Raspberry Pi robot runtime.

Primary target:

- Linux workstation on the same LAN as the robot

Secondary target:

- Windows operator who needs RViz2 against the real robot

Assumptions:

- Raspberry Pi hostname/user: `rosiyanin@<pi-ip>`
- Repo on Pi: `~/RTK2026`
- Main compose file: `docker/docker-compose.pi.yml`
- Runtime env file on Pi: `docker/pi.hardware.env`
- ROS 2 domain: `0`

## 1. Prepare Pi Runtime Env

On the Raspberry Pi:

```bash
cd ~/RTK2026
cp -n docker/pi.hardware.env.example docker/pi.hardware.env
nano docker/pi.hardware.env
```

Minimum important values:

```env
ARDUINO_SERIAL_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
ARDUINO_BAUDRATE=115200

USE_CAMERA=true
CAMERA_DRIVER=usb_cam
CAMERA_DEVICE=/dev/video0
CAMERA_FRAME_ID=camera_link
CAMERA_OPTICAL_FRAME_ID=camera_optical_link
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=10.0

USE_LIDAR=true
LIDAR_SERIAL_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_fc9214c1fa63ef119a7ae3a9c169b110-if00-port0
LIDAR_FRAME_ID=lidar_link

USE_REALSENSE_IMU=false
USE_BMI270_IMU=true
BMI270_BACKEND=spi
BMI270_FRAME_ID=imu_link
BMI270_PUBLISH_RATE_HZ=100.0
BMI270_ACCEL_X_SIGN=-1.0
BMI270_ACCEL_Y_SIGN=1.0
BMI270_ACCEL_Z_SIGN=-1.0
BMI270_GYRO_X_SIGN=-1.0
BMI270_GYRO_Y_SIGN=1.0
BMI270_GYRO_Z_SIGN=-1.0

USE_EKF=true
FOXGLOVE_PORT=8765
```

Useful mode switches:

```env
# Mapping mode
USE_SLAM_TOOLBOX=true

# Saved-map mode
USE_SLAM_TOOLBOX=false
MAP_YAML=/workspace/maps/<map-name>.yaml

# Route editor / lane manager mode
START_RVIZ=false
ENABLE_LANE_MANAGER=true
LOCALIZATION_USE_SIM_TIME=false
```

## 2. SLAM + Teleop + Foxglove

On the Raspberry Pi:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml up -d robot_runtime foxglove
```

Check runtime:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml ps
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml logs --tail=200 robot_runtime
```

Teleop:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml exec robot_runtime bash -lc '
source /opt/ros/jazzy/setup.bash &&
source /workspace/install/setup.bash &&
ros2 run teleop_twist_keyboard teleop_twist_keyboard
'
```

Foxglove:

```text
ws://<pi-ip>:8765
```

Main topics in mapping mode:

- `/scan`
- `/map`
- `/map_metadata`
- `/tf`
- `/tf_static`
- `/odometry/filtered`
- `/imu/data_raw`

## 3. Save Map After Mapping

On the Raspberry Pi:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml exec robot_runtime bash -lc '
source /opt/ros/jazzy/setup.bash &&
source /workspace/install/setup.bash &&
ros2 run nav2_map_server map_saver_cli -f /workspace/maps/my_map
'
```

Result:

- `/workspace/maps/my_map.yaml`
- `/workspace/maps/my_map.pgm`

These files are stored on the Pi under:

- `~/RTK2026/docker/saved_maps/`

## 4. Localization On Saved Map

For saved-map mode, set in `docker/pi.hardware.env`:

```env
USE_SLAM_TOOLBOX=false
MAP_YAML=/workspace/maps/my_map.yaml
LOCALIZATION_USE_SIM_TIME=false
```

Then start:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml up -d robot_runtime localization foxglove
```

Check:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml logs --tail=200 localization
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml exec robot_runtime bash -lc '
source /opt/ros/jazzy/setup.bash &&
source /workspace/install/setup.bash &&
ros2 topic list | grep -E "/map|/scan|/odom|/odometry/filtered|/tf"
'
```

## 5. Route Editor + Lane Manager On Real Robot

This is the runtime stack for:

- localization on a saved map
- perception
- route editor
- lane decision manager

Recommended Pi env values:

```env
USE_SLAM_TOOLBOX=false
MAP_YAML=/workspace/maps/my_map.yaml
LOCALIZATION_USE_SIM_TIME=false
START_RVIZ=false
ENABLE_LANE_MANAGER=true
```

Start the stack:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml up -d \
  robot_runtime localization perception route_editor_full foxglove
```

If you want browser-based RViz/noVNC from the Pi:

```text
http://<pi-ip>:6084/vnc.html
```

Check lane stack:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml logs --tail=200 route_editor_full
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml logs --tail=200 localization
```

## 6. Linux Workstation With Native RViz2

This is the preferred operator setup.

Requirements:

- Linux machine on the same LAN as the robot
- ROS 2 Jazzy installed locally
- multicast/firewall not blocked on the LAN

Environment:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
```

### 6.1. RViz2 for SLAM View

Use:

```bash
rviz2 -d /path/to/RTK2026/docker/local_rviz/rtk2026_slam.rviz
```

### 6.2. RViz2 for Route / Lane Work

Use:

```bash
rviz2 -d /path/to/RTK2026/src/rtk2026_route_nav/rviz/rtk2026_route_editor.rviz
```

Expected topics:

- `/map`
- `/scan`
- `/tf`
- `/tf_static`
- `/odometry/filtered`
- `/camera/image_raw`

## 7. Windows Operator

Priority remains Linux.

For Windows there are 3 practical options.

### 7.1. Recommended Windows Fallback: Browser noVNC

Use the Pi-hosted route editor:

```text
http://<pi-ip>:6084/vnc.html
```

This avoids DDS and native Windows ROS issues completely.

### 7.2. Good Windows Fallback: Foxglove

Connect to:

```text
ws://<pi-ip>:8765
```

### 7.3. Native RViz2 On Windows

Use only if the operator really needs native RViz2.

Requirements:

- native ROS 2 Jazzy on Windows
- same Layer-2 LAN as the robot
- Windows firewall allows ROS 2 DDS traffic
- do not use WSL2 as the primary DDS client for this setup

Environment in `cmd.exe`:

```bat
set ROS_DOMAIN_ID=0
set ROS_LOCALHOST_ONLY=0
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Then run RViz2:

```bat
rviz2 -d C:\path\to\RTK2026\src\rtk2026_route_nav\rviz\rtk2026_route_editor.rviz
```

If DDS discovery is unstable on Windows, use:

- Foxglove
- or Pi noVNC route editor

## 8. Stop Commands

Stop only mapping/runtime stack:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml stop robot_runtime foxglove
```

Stop saved-map + lane stack:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml stop \
  robot_runtime localization perception route_editor_full foxglove
```

Stop and remove containers:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml down
```

## 9. Fast Diagnostics

Runtime:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml ps
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml logs --tail=200 robot_runtime
```

ROS graph from the Pi:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml exec robot_runtime bash -lc '
source /opt/ros/jazzy/setup.bash &&
source /workspace/install/setup.bash &&
ros2 topic list
'
```

Critical checks:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml exec robot_runtime bash -lc '
source /opt/ros/jazzy/setup.bash &&
source /workspace/install/setup.bash &&
ros2 topic info -v /scan &&
echo --- &&
ros2 topic info -v /imu/data_raw &&
echo --- &&
ros2 topic info -v /odometry/filtered
'
```

For TF:

```bash
cd ~/RTK2026/docker
sudo docker compose --env-file pi.hardware.env -f docker-compose.pi.yml exec robot_runtime bash -lc '
source /opt/ros/jazzy/setup.bash &&
source /workspace/install/setup.bash &&
ros2 run tf2_ros tf2_echo base_footprint lidar_link
'
```
