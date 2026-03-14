# RTK2026 ROS2 Humble - build and run
# Used for: bringup, simulation with Isaac Sim (use_sim_time), SLAM + Nav2 + RViz in Docker (see docs/ISAAC_ROS_SLAM.md)
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# Install build and launch dependencies (без обязательных Gazebo-пакетов)
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
    && rm -rf /var/lib/apt/lists/*

# Опциональные Gazebo-пакеты: могут отсутствовать на некоторых архитектурах (arm64)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros2-control \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros \
    || echo "Gazebo packages not available for this architecture, skipping."

RUN pip3 install -U rosdep && rosdep init || true
RUN pip3 install pyserial

WORKDIR /workspace

# Copy workspace source (build context is repo root)
COPY src /workspace/src

# Install package dependencies and build
# В Docker-образе пропускаем вендорный пакет diff_robot (требует gazebo_ros, не установленный из deb)
RUN . /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install -y -i --from-paths src --skip-keys "rtk2026 rtk2026_description rtk2026_bringup rtk2026_interfaces rtk2026_driver rtk2026_base rtk2026_localization rtk2026_slam rtk2026_navigation rtk2026_simulation rtk2026_peripherals diff_robot rtk2026_nav2_explorer" || true && \
    colcon build --symlink-install --packages-skip diff_robot

# Source workspace and run default launch
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_DOMAIN_ID=0

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch rtk2026_bringup rtk2026.launch.py"]
