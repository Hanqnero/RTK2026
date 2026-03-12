cmake_minimum_required(VERSION 3.8)
project(rtk2026_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
