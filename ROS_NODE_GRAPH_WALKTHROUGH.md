# Разбор ROS2 node graph

Файл графа: [`ros_node_graph.png`](./ros_node_graph.png)

Граф снят с текущей Docker-симуляции `RTK2026-2`, где SLAM работает через нашу одометрию:

```text
/joint_states -> sim_encoder -> /encoder_report -> wheel_odometry -> /odom + TF
/scan -> slam_toolbox -> /map
```

## Как читать граф

В `rqt_graph`:

```text
овалы = ROS nodes
прямоугольники = ROS topics
node -> topic = нода публикует топик
topic -> node = нода подписана на топик
```

`rqt_graph` показывает в основном publish/subscribe связи по топикам. Services и actions на этом графе не являются основной частью отображения.

В текущем запуске:

```text
actions: нет
services: есть, но это Gazebo/SLAM/parameter services, не основная цепочка движения
```

На PNG могут быть служебные артефакты от снятия графа:

```text
/_ros2cli_daemon_...
/rqt_graph_exporter
```

Это не часть алгоритма робота.

## Основной путь данных

Команда движения:

```text
/cmd_vel
  -> /diff_drive
  -> физика Gazebo, wheel joints
```

Симуляция энкодеров:

```text
/joint_state_publisher
  -> /joint_states
  -> /sim_encoder
  -> /encoder_report
  -> /wheel_odometry
  -> /odom
  -> /tf: odom -> base_footprint
```

Лидар и SLAM:

```text
/lidar_plugin
  -> /scan
  -> /slam_toolbox
  -> /map
  -> /tf: map -> odom
```

URDF и TF:

```text
/robot_state_publisher
  -> /robot_description
  -> /tf_static: base_footprint -> base_link, base_link -> lidar_link, ...
  -> /tf: wheel joints from /joint_states
```

Визуализация:

```text
/foxglove_bridge
  читает /map, /tf, /tf_static, /robot_description и visualization topics
```

## Nodes

`/gazebo`

Симулятор. Публикует `/clock` и `/performance_metrics`, держит Gazebo services вроде `/reset_world`, `/spawn_entity`, `/pause_physics`.

`/diff_drive`

Gazebo plugin дифференциального привода. Читает `/cmd_vel` и крутит wheel joints. В текущей конфигурации не публикует `/odom` и не публикует odom TF.

`/joint_state_publisher`

Gazebo plugin, который публикует `/joint_states` по wheel joints. Нужен как источник симуляционных углов колёс.

`/sim_encoder`

Наш симуляционный адаптер. Читает `/joint_states`, переводит углы колёс в накопленные тики энкодеров и публикует `/encoder_report`.

`/wheel_odometry`

Наша одометрия. Читает `/encoder_report`, публикует `/odom` и TF `odom -> base_footprint`.

`/robot_state_publisher`

Читает URDF и `/joint_states`, публикует `/robot_description`, `/tf`, `/tf_static`. Даёт связи `base_footprint -> base_link -> lidar_link/wheels/front_caster_link`.

`/lidar_plugin`

Gazebo plugin лидара. Публикует `/scan`.

`/slam_toolbox`

Читает `/scan` и TF, строит карту. Публикует `/map`, `/map_metadata`, `/pose`, visualization topics и TF `map -> odom`.

`/transform_listener_impl_...`

Внутренний TF listener от `slam_toolbox`. Читает `/tf` и `/tf_static`.

`/foxglove_bridge`

Мост для Foxglove. В алгоритме движения и SLAM не участвует. Читает визуализационные топики и может публиковать интерактивные топики из UI.

## Topics

### `/cmd_vel`

```text
type: geometry_msgs/msg/Twist
publisher: сейчас нет, появляется при teleop/Foxglove/manual command
subscriber: /diff_drive
```

Команда скорости робота. `linear.x` задаёт движение вперёд/назад, `angular.z` задаёт поворот. В симуляции этот топик читает Gazebo `diff_drive`.

### `/joint_states`

```text
type: sensor_msgs/msg/JointState
publisher: /joint_state_publisher
subscribers: /sim_encoder, /robot_state_publisher
```

Состояния joint'ов колёс: имена, углы, скорости. Для нас это симуляционный аналог данных от колёсных энкодеров.

### `/encoder_report`

```text
type: rtk2026_interfaces/msg/EncoderReport
publisher: /sim_encoder
subscriber: /wheel_odometry
```

Наш формат отчёта энкодеров. В симуляции создаётся из `/joint_states`; на реальном роботе должен приходить из driver/Arduino-цепочки.

### `/odom`

```text
type: nav_msgs/msg/Odometry
publisher: /wheel_odometry
direct subscribers: нет
```

Одометрия, рассчитанная нашим модулем по `/encoder_report`. Даже если у `/odom` нет прямых subscribers, это нормально: `slam_toolbox` использует не сам топик `/odom`, а TF `odom -> base_footprint` из `/tf`.

### `/scan`

```text
type: sensor_msgs/msg/LaserScan
publisher: /lidar_plugin
subscriber: /slam_toolbox
```

Сырые данные 2D-лидара. Это основной сенсорный вход для `slam_toolbox`.

### `/map`

```text
type: nav_msgs/msg/OccupancyGrid
publisher: /slam_toolbox
subscribers: /slam_toolbox, /foxglove_bridge
```

Построенная occupancy grid карта. `foxglove_bridge` читает её для отображения в Foxglove.

### `/map_metadata`

```text
type: nav_msgs/msg/MapMetaData
publisher: /slam_toolbox
subscribers: нет
```

Метаданные карты: resolution, width, height, origin.

### `/pose`

```text
type: geometry_msgs/msg/PoseWithCovarianceStamped
publisher: /slam_toolbox
subscribers: нет
```

Оценка позы от `slam_toolbox`. В текущей цепочке основная поза для визуализации обычно читается через TF, а не через этот топик.

### `/tf`

```text
type: tf2_msgs/msg/TFMessage
publishers: /wheel_odometry, /slam_toolbox, /robot_state_publisher
subscribers: /foxglove_bridge, /transform_listener_impl_...
```

Динамические TF-связи. Основные:

```text
map -> odom              от /slam_toolbox
odom -> base_footprint   от /wheel_odometry
base_link -> wheels      от /robot_state_publisher по /joint_states
```

### `/tf_static`

```text
type: tf2_msgs/msg/TFMessage
publisher: /robot_state_publisher
subscribers: /foxglove_bridge, /transform_listener_impl_...
```

Статические TF-связи из URDF. Основные:

```text
base_footprint -> base_link
base_link -> lidar_link
base_link -> front_caster_link
```

### `/robot_description`

```text
type: std_msgs/msg/String
publisher: /robot_state_publisher
subscriber: /foxglove_bridge
```

URDF-модель робота в виде строки. Foxglove использует её, чтобы отображать модель робота.

### `/clock`

```text
type: rosgraph_msgs/msg/Clock
publisher: /gazebo
subscribers: Gazebo plugins, /sim_encoder, /wheel_odometry, /slam_toolbox, /robot_state_publisher
```

Sim time. Все ноды с `use_sim_time:=true` используют этот топик вместо системного времени.

### `/performance_metrics`

```text
type: gazebo_msgs/msg/PerformanceMetrics
publisher: /gazebo
subscribers: нет
```

Метрики производительности Gazebo. Для алгоритма робота не нужен.

### `/rosout`

```text
type: rcl_interfaces/msg/Log
publishers: почти все ноды
subscriber: /foxglove_bridge
```

Стандартный ROS-топик логов. Нужен для диагностики, не участвует в алгоритме движения.

### `/parameter_events`

```text
type: rcl_interfaces/msg/ParameterEvent
publishers: почти все ноды
subscribers: почти все ноды и /foxglove_bridge
```

Служебный ROS2-топик событий параметров. На графе создаёт много шумных связей, но не является частью алгоритма SLAM/одометрии.

### `/slam_toolbox/scan_visualization`

```text
type: sensor_msgs/msg/LaserScan
publisher: /slam_toolbox
subscriber: /foxglove_bridge
```

Визуализационный scan от `slam_toolbox`. Это не сырой лидар; сырой лидар находится в `/scan`.

### `/slam_toolbox/graph_visualization`

```text
type: visualization_msgs/msg/MarkerArray
publisher: /slam_toolbox
subscriber: /foxglove_bridge
```

Визуализация pose graph/SLAM graph. Нужна для отладки SLAM в визуализаторе.

### `/slam_toolbox/update`

```text
type: visualization_msgs/msg/InteractiveMarkerUpdate
publisher: /slam_toolbox
subscribers: нет
```

Служебная визуализация interactive markers от `slam_toolbox`.

### `/slam_toolbox/feedback`

```text
type: visualization_msgs/msg/InteractiveMarkerFeedback
publisher: нет
subscriber: /slam_toolbox
```

Feedback от interactive markers. В текущем запуске publisher отсутствует, поэтому на основной алгоритм не влияет.

### `/clicked_point`

```text
type: geometry_msgs/msg/PointStamped
publisher: /foxglove_bridge
subscribers: нет
```

Точка, которую пользователь может кликнуть в Foxglove. Сейчас никто её не читает.

### `/initialpose`

```text
type: geometry_msgs/msg/PoseWithCovarianceStamped
publisher: /foxglove_bridge
subscribers: нет
```

Начальная поза из UI. Сейчас нет ноды, которая её использует.

### `/move_base_simple/goal`

```text
type: geometry_msgs/msg/PoseStamped
publisher: /foxglove_bridge
subscribers: нет
```

Цель навигации из UI. Сейчас Nav2/action server не запущен, поэтому этот топик никто не читает.

## Чего быть не должно в рабочей схеме

```text
/gazebo_odom
/rtk_odom
```

Если эти топики появились, значит снова включился режим сравнения или Gazebo odom начал участвовать в графе. В рабочей схеме SLAM должен идти через `/odom` от `/wheel_odometry`.

## Минимальная цепочка для проверки

Если граф слишком шумный, проверяй именно эти связи:

```text
/cmd_vel -> /diff_drive
/joint_state_publisher -> /joint_states -> /sim_encoder -> /encoder_report -> /wheel_odometry
/wheel_odometry -> /odom
/lidar_plugin -> /scan -> /slam_toolbox
/slam_toolbox -> /map
/robot_state_publisher -> /tf_static
/wheel_odometry -> /tf
/slam_toolbox -> /tf
```

И TF:

```text
map -> odom -> base_footprint -> base_link -> lidar_link
```
