cmake_minimum_required(VERSION 3.8)

# Имя ROS 2-пакета.
# Оно должно совпадать с <name> в package.xml
project(rtk2026_description)

# Подключаем стандартную систему сборки ROS 2 для CMake-пакетов
find_package(ament_cmake REQUIRED)

# Устанавливаем ресурсные директории пакета
install(DIRECTORY urdf config launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

# quantize_joint_states.py остаётся диагностическим инструментом для сравнения
# идеальных joint states Gazebo с разрешением реальных энкодеров.
# Преобразование /cmd_vel в скорости колёс выполняет штатный
# diff_drive_controller.
install(PROGRAMS scripts/quantize_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
)
# Завершаем описание ament-пакета.
ament_package()
