cmake_minimum_required(VERSION 3.8)

project(rtk2026_slam)

find_package(ament_cmake REQUIRED)

# Пакет SLAM является единственным владельцем параметров slam_toolbox.
# Bringup и будущий симуляционный launch получают этот файл через ament index.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
