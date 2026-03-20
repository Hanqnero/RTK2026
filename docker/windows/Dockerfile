# RTK2026 ROS2 Humble -- Windows (amd64)
# Gazebo + diff_robot + RTK-стек (SLAM, Nav2, explorer). Полная симуляция на трассе.
# Сборка: docker build -t rtk2026:latest -f docker/windows/Dockerfile .
# Запуск: .\scripts\run_rtk2026_diff_robot.ps1 -Build -Explore -World track
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# --- Слой 1: системные и ROS-пакеты (меняется редко) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-std-msgs \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
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
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U rosdep && rosdep init || true
RUN pip3 install pyserial

WORKDIR /workspace

# --- Слой 2: rosdep (меняется только при изменении package.xml) ---
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
COPY src/diff_robot/package.xml             src/diff_robot/package.xml

RUN . /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install -y -i --from-paths src \
      --skip-keys "rtk2026 rtk2026_description rtk2026_bringup rtk2026_interfaces \
                   rtk2026_driver rtk2026_base rtk2026_localization rtk2026_slam \
                   rtk2026_navigation rtk2026_simulation rtk2026_peripherals \
                   diff_robot rtk2026_nav2_explorer" || true

# --- Слой 3: исходники + сборка (инвалидируется при изменении кода) ---
COPY src /workspace/src

RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_DOMAIN_ID=0

CMD ["/bin/bash", "-c", \
     "source /opt/ros/humble/setup.bash && \
      source /workspace/install/setup.bash && \
      ros2 launch rtk2026_bringup rtk2026.launch.py"]
