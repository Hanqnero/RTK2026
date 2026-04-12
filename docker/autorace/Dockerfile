# RTK2026 Autorace -- Windows (amd64)
# Компактный образ только для TurtleBot3 Autorace симуляции.
# Без Nav2 / SLAM / robot-localization — только Gazebo + CV + авторейс-пайплайн.
#
# Сборка:
#   docker build -t rtk2026-autorace:latest -f docker/autorace/Dockerfile .
#
# Запуск:
#   .\scripts\run_rtk2026_diff_robot.ps1 -Autorace -NoMap -World autorace

FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# --- Слой 1: системные и ROS-пакеты ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-std-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-opencv \
    python3-opencv \
    libopencv-dev \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-teleop-twist-keyboard \
    ros-humble-slam-toolbox \
    ros-humble-rqt \
    ros-humble-rqt-image-view \
    ros-humble-rqt-common-plugins \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install -U rosdep && rosdep init || true

WORKDIR /workspace

# --- Слой 2: rosdep (меняется только при изменении package.xml) ---
COPY src/diff_robot/package.xml              src/diff_robot/package.xml
COPY src/rtk2026_simulation/package.xml      src/rtk2026_simulation/package.xml
COPY src/rtk2026_peripherals/package.xml     src/rtk2026_peripherals/package.xml

RUN . /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install -y -i --from-paths src \
      --skip-keys "diff_robot rtk2026_simulation rtk2026_peripherals" || true

# --- Слой 3: исходники нужных пакетов ---
COPY src/diff_robot             src/diff_robot
COPY src/rtk2026_simulation     src/rtk2026_simulation
COPY src/rtk2026_peripherals    src/rtk2026_peripherals
COPY src/turtlebot3_autorace    src/turtlebot3_autorace

RUN . /opt/ros/humble/setup.bash && \
    rosdep install -y -i --from-paths src \
      --skip-keys "diff_robot rtk2026_simulation rtk2026_peripherals \
                   turtlebot3_autorace turtlebot3_autorace_camera \
                   turtlebot3_autorace_detect turtlebot3_autorace_mission" || true

RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
      --packages-select \
        diff_robot \
        rtk2026_simulation \
        rtk2026_peripherals \
        turtlebot3_autorace \
        turtlebot3_autorace_camera \
        turtlebot3_autorace_detect \
        turtlebot3_autorace_mission

# --- Слой 4: патчи autorace ---
COPY docker/patches /workspace/docker/patches

RUN python3 /workspace/docker/patches/patch_detect_lane_publish_neutral.py \
      || echo "patch_detect_lane_publish_neutral: already applied or skipped"
RUN python3 /workspace/docker/patches/patch_control_lane_neutral.py \
      || echo "patch_control_lane_neutral: already applied or skipped"
RUN python3 /workspace/docker/patches/patch_avoid_construction_params.py \
      || echo "patch_avoid_construction_params: already applied or skipped"

RUN cp /workspace/docker/patches/lane.yaml \
       /workspace/install/turtlebot3_autorace_detect/share/turtlebot3_autorace_detect/param/lane/lane.yaml

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["/bin/bash", "-c", \
     "source /opt/ros/humble/setup.bash && \
      source /workspace/install/setup.bash && \
      ros2 launch rtk2026_simulation rtk2026_autorace.launch.py \
        world:=/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_autorace_2020.world \
        x:=0.8 y:=-1.747 z:=0.05 use_sim_time:=true use_slam:=false"]
