cmake_minimum_required(VERSION 3.8)

project(rtk2026_observability)

find_package(ament_cmake REQUIRED)

# Конфигурации и launch-файлы диагностики.
install(
  DIRECTORY
    config
    launch
  DESTINATION
    share/${PROJECT_NAME}
)
# Универсальные ноды диагностики ROS 2 graph, TF, сенсоров и локализации.
install(
  PROGRAMS
    scripts/topic_monitor.py
    scripts/node_monitor.py
    scripts/tf_time_monitor.py
    scripts/sensor_monitor.py
    scripts/localization_monitor.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# Универсальная rosbag2-запись без привязки к миру, сенсорам или SLAM.
install(
  PROGRAMS
    scripts/record_experiment.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME
    record_experiment
)

# Независимый набор автоматически нумеруемых TF-поз.
install(
  PROGRAMS
    scripts/pose_recorder.py
  DESTINATION
    lib/${PROJECT_NAME}
  RENAME
    pose_recorder
)

ament_package()
