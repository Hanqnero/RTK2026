cmake_minimum_required(VERSION 3.8)

project(rtk2026_tracked_sim)

find_package(ament_cmake REQUIRED)

# Пакет хранит только описание модели.
# Общий запуск находится в rtk2026_bringup/sim_slam_launch.py.
install(
  DIRECTORY
    models
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  FILES
    README.md
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
