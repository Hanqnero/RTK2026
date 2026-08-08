cmake_minimum_required(VERSION 3.8)

project(rtk2026_nav2)

find_package(ament_cmake REQUIRED)

# Пакет состоит из настройки и запуска: исполняемого кода в нём нет.
install(
  DIRECTORY
    behavior_trees
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
