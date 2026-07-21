cmake_minimum_required(VERSION 3.8)

project(rtk2026_bringup)

find_package(ament_cmake REQUIRED)

# Bringup хранит только сценарии запуска.
# Конфигурации принадлежат пакетам соответствующих узлов:
#   - Arduino bridge: rtk2026_driver/config;
#   - SLAM:           rtk2026_slam/config.
install(
  DIRECTORY
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
