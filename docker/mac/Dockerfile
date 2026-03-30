# RTK2026 ROS2 Humble -- Mac (Apple Silicon / arm64)
# Calibration image: RealSense + camera_calibration + foxglove_bridge
#
# Build: docker buildx build --platform linux/arm64 \
#          -t rtk2026-calib:latest -f docker/mac/Dockerfile .
# Run:   scripts/run_calibration_mac.sh [intrinsic|extrinsic]
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Layer 1: ROS2 Humble + calibration packages ---
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl gnupg ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential cmake git pkg-config \
        libusb-1.0-0-dev libudev-dev libssl-dev \
        libwebsocketpp-dev libasio-dev nlohmann-json3-dev \
        python3-pip python3-rosdep python3-colcon-common-extensions \
        python3-opencv \
        ros-humble-ros-base \
        ros-humble-std-msgs \
        ros-humble-sensor-msgs \
        ros-humble-image-transport \
        ros-humble-image-proc \
        ros-humble-cv-bridge \
        ros-humble-camera-calibration \
        ros-humble-camera-info-manager \
        ros-humble-diagnostic-updater \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# --- Layer 2: librealsense2 from source (no arm64 apt package) ---
RUN git clone --depth=1 \
    https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && mkdir -p /tmp/librealsense/build \
    && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_WITH_CUDA=OFF \
        -DFORCE_RSUSB_BACKEND=ON \
    && cmake --build /tmp/librealsense/build -j$(nproc) \
    && cmake --install /tmp/librealsense/build \
    && rm -rf /tmp/librealsense \
    && ldconfig

WORKDIR /workspace

# --- Layer 3: rosdep (invalidated only when package.xml changes) ---
COPY src/rtk2026_interfaces/package.xml       src/rtk2026_interfaces/package.xml
COPY src/rtk2026_peripherals/package.xml      src/rtk2026_peripherals/package.xml
COPY src/turtlebot3_autorace/turtlebot3_autorace_camera/package.xml \
     src/turtlebot3_autorace/turtlebot3_autorace_camera/package.xml

RUN . /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install -y -i --from-paths src \
      --skip-keys "rtk2026_interfaces rtk2026_peripherals turtlebot3_autorace_camera" || true

# --- Layer 4: source + build ---
RUN git clone --depth=1 --branch ros2-master \
    https://github.com/IntelRealSense/realsense-ros.git \
    /workspace/src/realsense-ros

RUN git clone --depth=1 \
    https://github.com/foxglove/foxglove-sdk.git \
    /workspace/src/foxglove-sdk

COPY src /workspace/src

RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_PREFIX_PATH=/usr/local \
      --packages-select \
        rtk2026_interfaces \
        rtk2026_peripherals \
        turtlebot3_autorace_camera \
        realsense2_camera_msgs \
        realsense2_camera \
        foxglove_bridge \
        foxglove_msgs

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["/bin/bash", "-c", \
     "source /opt/ros/humble/setup.bash && \
      source /workspace/install/setup.bash && \
      ros2 launch rtk2026_peripherals calibration.launch.py"]
