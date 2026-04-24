# Raspberry Pi Runtime

## What URDF Is Used

The real robot runtime uses:

- `src/rtk2026_description/urdf/rtk2026.urdf.xacro`

It is loaded by:

- `src/rtk2026_description/launch/description.launch.py`
- `src/rtk2026_bringup/launch/robot_runtime.launch.py`

The Gazebo-specific file:

- `src/rtk2026_description/urdf/rtk2026_gazebo.urdf.xacro`

is only for simulation.

## Frames Present In Real URDF

The real URDF currently contains:

- `base_footprint`
- `base_link`
- `lidar_link`
- `camera_link`
- `camera_optical_link`
- `imu_link`

`camera_optical_link` follows REP-103 optical frame convention.

## Hardware Port Configuration

Copy:

- `docker/pi.hardware.env.example`

to:

- `docker/pi.hardware.env`

and edit it on the Raspberry Pi.

Main variables:

- `ARDUINO_SERIAL_PORT`
- `ARDUINO_BAUDRATE`
- `USE_CAMERA`
- `CAMERA_DRIVER`
- `CAMERA_DEVICE`
- `CAMERA_FRAME_ID`
- `CAMERA_OPTICAL_FRAME_ID`
- `CAMERA_WIDTH`
- `CAMERA_HEIGHT`
- `CAMERA_FPS`
- `USE_LIDAR`
- `LIDAR_SERIAL_PORT`
- `LIDAR_FRAME_ID`
- `USE_REALSENSE_IMU`
- `USE_EKF`
- `IMU_DEVICE`
- `IMU_FRAME_ID`

Auto-detect helper on the Raspberry Pi:

```bash
cd ~/RTK2026
bash docker/scripts/detect_pi_hardware.sh
```

To write detected values directly into `docker/pi.hardware.env`:

```bash
cd ~/RTK2026
bash docker/scripts/detect_pi_hardware.sh --write
```

BMI270 preparation notes:

- `docker/BMI270_INTEGRATION.md`
- `docker/REAL_ROBOT_RUNBOOK.md`

Run compose with:

```bash
docker compose --env-file docker/pi.hardware.env -f docker/docker-compose.pi.yml up -d
```

For Foxglove-based SLAM testing from Mac:

```bash
docker compose --env-file docker/pi.hardware.env -f docker/docker-compose.pi.yml up -d \
  robot_runtime slam foxglove
```

Then connect Foxglove on Mac to:

```text
ws://<pi-ip>:8765
```

## What Is Already Wired

- `robot_runtime` passes `arduino_serial_port` and `arduino_baudrate` into `rtk2026_bringup/robot_runtime.launch.py`
- `arduino_bridge` uses `/cmd_vel` and sends only target `linear/angular` velocities to Arduino
- Arduino converts them to wheel targets internally
- CloverCam-style USB camera is launched through `usb_cam` to `/camera/image_raw`
- Slamtec lidar is launched through `sllidar_ros2` to `/scan`
- RealSense is launched in IMU-only mode to provide `/camera/imu`
- `robot_localization` EKF fuses wheel `/odom` with `/camera/imu` and publishes `odom -> base_footprint`
- `foxglove_bridge` can publish the ROS graph over WebSocket for Mac-side Foxglove without DDS on macOS

## What Is Not Yet Implemented

What is still not done:

- camera/lidar/imu runtime diagnostics and recording helpers are not yet added
