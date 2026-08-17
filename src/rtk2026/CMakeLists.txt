cmake_minimum_required(VERSION 3.8)

project(rtk2026)

find_package(ament_cmake REQUIRED)

# Метапакет не устанавливает исполняемые файлы или конфигурации.
# Его задача — собрать зависимости проекта под одним именем.
ament_package()
