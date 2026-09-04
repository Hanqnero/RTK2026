cmake_minimum_required(VERSION 3.8)

# Имя ROS 2-пакета.
# Оно должно совпадать с <name> в package.xml
project(rtk2026_description)

# Подключаем стандартную систему сборки ROS 2 для CMake-пакетов
find_package(ament_cmake REQUIRED)

# Устанавливаем ресурсные директории пакета
install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

# meshes/ и worlds/ на роботе не нужны: описание там разворачивается без
# геометрии, а миры к реальному роботу отношения не имеют. На Pi эти
# каталоги не приезжают (см. EXCLUDES в pi/tools/sync_to_pi.sh), поэтому
# установка выполняется только при их наличии - иначе сборка образа
# упала бы на отсутствующем каталоге.
foreach(optional_asset_dir meshes worlds)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${optional_asset_dir}")
    install(DIRECTORY ${optional_asset_dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

# quantize_joint_states.py остаётся диагностическим инструментом для сравнения
# идеальных joint states Gazebo с разрешением реальных энкодеров.
# Преобразование /cmd_vel в скорости колёс выполняет штатный
# diff_drive_controller.
install(PROGRAMS scripts/quantize_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
)
# Завершаем описание ament-пакета.
ament_package()
