# RTK2026 ROS2 Humble -- Raspberry Pi (arm64)
# Headless: без Gazebo и RViz. Драйвер (arduino_bridge), база, SLAM, Nav2, Foxglove.
# Сборка: docker buildx build --platform linux/arm64 -t rtk2026:latest -f docker/pi/Dockerfile .
# Запуск:  docker run -d --name rtk2026 --privileged -v /dev:/dev --network host rtk2026:latest
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Слой 1: ROS2 Humble + все пакеты ---
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl gnupg ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        pkg-config \
        libusb-1.0-0-dev \
        libudev-dev \
        libssl-dev \
        libwebsocketpp-dev \
        libasio-dev \
        nlohmann-json3-dev \
        python3-pip \
        python3-rosdep \
        python3-colcon-common-extensions \
        ros-humble-ros-base \
        ros-humble-std-msgs \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-launch \
        ros-humble-launch-ros \
        ros-humble-robot-localization \
        ros-humble-slam-toolbox \
        ros-humble-nav2-bringup \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-diff-drive-controller \
        ros-humble-joint-state-broadcaster \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-teleop-twist-keyboard \
        ros-humble-usb-cam \
        ros-humble-image-transport \
        ros-humble-camera-calibration \
        ros-humble-diagnostic-updater \
        ros-humble-rosx-introspection \
        usbutils \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# --- Слой 2: librealsense2 из исходников (нет надёжного apt для arm64) ---
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

RUN pip3 install pyserial "numpy<2" onnxruntime

WORKDIR /workspace

# --- Слой 2b: vision пакеты для lane detection (отдельный слой — не трогает librealsense кеш) ---
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-image-proc \
        ros-humble-cv-bridge \
        python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# --- Слой 3: rosdep (меняется только при изменении package.xml) ---
COPY src/rtk2026/package.xml                src/rtk2026/package.xml
COPY src/rtk2026_interfaces/package.xml     src/rtk2026_interfaces/package.xml
COPY src/rtk2026_description/package.xml    src/rtk2026_description/package.xml
COPY src/rtk2026_driver/package.xml         src/rtk2026_driver/package.xml
COPY src/rtk2026_base/package.xml           src/rtk2026_base/package.xml
COPY src/rtk2026_bringup/package.xml        src/rtk2026_bringup/package.xml
COPY src/rtk2026_localization/package.xml   src/rtk2026_localization/package.xml
COPY src/rtk2026_slam/package.xml           src/rtk2026_slam/package.xml
COPY src/rtk2026_navigation/package.xml     src/rtk2026_navigation/package.xml
COPY src/rtk2026_simulation/package.xml     src/rtk2026_simulation/package.xml
COPY src/rtk2026_peripherals/package.xml    src/rtk2026_peripherals/package.xml
COPY src/rtk2026_nav2_explorer/package.xml  src/rtk2026_nav2_explorer/package.xml
COPY src/turtlebot3_autorace/turtlebot3_autorace_camera/package.xml \
     src/turtlebot3_autorace/turtlebot3_autorace_camera/package.xml
COPY src/turtlebot3_autorace/turtlebot3_autorace_detect/package.xml \
     src/turtlebot3_autorace/turtlebot3_autorace_detect/package.xml
COPY src/turtlebot3_autorace/turtlebot3_autorace_mission/package.xml \
     src/turtlebot3_autorace/turtlebot3_autorace_mission/package.xml

RUN . /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install -y -i --from-paths src \
      --skip-keys "rtk2026 rtk2026_description rtk2026_bringup rtk2026_interfaces \
                   rtk2026_driver rtk2026_base rtk2026_localization rtk2026_slam \
                   rtk2026_navigation rtk2026_simulation rtk2026_peripherals \
                   diff_robot rtk2026_nav2_explorer" || true

# --- Слой 4: исходники + сборка (инвалидируется при изменении кода) ---
# sllidar_ros2 (Slamtec) нет в стандартных apt — клонируем исходник
RUN git clone --depth=1 \
    https://github.com/Slamtec/sllidar_ros2.git \
    /workspace/src/sllidar_ros2

# realsense-ros: ROS2 обёртка над librealsense2 (D435i)
RUN git clone --depth=1 --branch ros2-master \
    https://github.com/IntelRealSense/realsense-ros.git \
    /workspace/src/realsense-ros

# foxglove_bridge for ROS 2 lives in foxglove-sdk/ros/src
RUN git clone --depth=1 \
    https://github.com/foxglove/foxglove-sdk.git \
    /tmp/foxglove-sdk \
    && cp -r /tmp/foxglove-sdk/ros/src/foxglove_bridge /workspace/src/foxglove_bridge \
    && cp -r /tmp/foxglove-sdk/ros/src/foxglove_msgs /workspace/src/foxglove_msgs \
    && rm -rf /tmp/foxglove-sdk

COPY src /workspace/src

# Собираем только пакеты для Pi (без Gazebo/симуляции)
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_PREFIX_PATH=/usr/local \
      --packages-select \
        rtk2026_interfaces \
        rtk2026_description \
        rtk2026_driver \
        rtk2026_base \
        rtk2026_bringup \
        rtk2026_localization \
        rtk2026_slam \
        rtk2026_navigation \
        rtk2026_peripherals \
        rtk2026_nav2_explorer \
        sllidar_ros2 \
        realsense2_camera_msgs \
        realsense2_camera \
        foxglove_bridge \
        foxglove_msgs \
        turtlebot3_autorace_camera \
        turtlebot3_autorace_detect \
        turtlebot3_autorace_mission

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_DOMAIN_ID=0

CMD ["/bin/bash", "-c", \
     "source /opt/ros/humble/setup.bash && \
      source /workspace/install/setup.bash && \
      ros2 launch rtk2026_bringup pi_full.launch.py"]
