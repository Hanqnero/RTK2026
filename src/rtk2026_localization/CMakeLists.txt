cmake_minimum_required(VERSION 3.8)

project(rtk2026_localization)

find_package(ament_cmake REQUIRED)

# Пакет не компилирует собственный фильтр: он устанавливает проверенные
# конфигурации и launch-обвязку для robot_localization и Nav2 AMCL.
install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
