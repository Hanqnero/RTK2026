# RTK2026: фиксированная модель робота и отдельный стенд дифференциального привода

В пакете намеренно разделены две разные задачи.

## 1. Модель реального робота для RViz и TF

Файл:

```text
urdf/rtk2026_real.urdf.xacro
```

В этой модели ходовая часть является жёсткой геометрией:

```text
base_footprint
  -> base_link
    -> wheel_link
    -> imu_link
    -> lidar_link -> lidar_frame
    -> camera_link -> camera_optical_frame
```

`wheel_joint` специально имеет тип `fixed`. В модели нет `left_wheel_joint`,
`right_wheel_joint`, интерфейсов энкодеров, `ros2_control` и Gazebo-контроллеров.
Команды двигателям передаются аппаратным драйвером непосредственно на реальном
роботе; URDF в этом тракте только задаёт геометрию, инерции и дерево фреймов.

Все суставы модели фиксированные, поэтому для её публикации нужен только
`robot_state_publisher`; `joint_state_publisher` и `/joint_states` не требуются.
Положение робота в мировой системе координат должно публиковаться отдельным узлом
как TF, например `odom -> base_footprint`, если оно вообще требуется визуализации.

Запуск:

```bash
ros2 launch rtk2026_description display.launch.py use_meshes:=false
```

После копирования STL:

```bash
ros2 launch rtk2026_description display.launch.py use_meshes:=true
```

## 2. Отдельный Gazebo-стенд дифференциального привода

Файл:

```text
urdf/rtk2026_diff_drive_sim.urdf.xacro
```

Только в этой модели существуют вращаемые суставы:

```text
left_wheel_joint  : continuous
right_wheel_joint : continuous
```

Они подключены к `ros2_control` и штатному `diff_drive_controller`. Контроллер
получает положения колёс как обратную связь энкодеров, рассчитывает `/odom` и
публикует TF `odom -> base_footprint`. Этот стенд не является механической моделью
реального робота для RViz и не должен включаться в `rtk2026_real`.

Тракт тестирования:

```text
/cmd_vel
  -> /diff_drive_controller/cmd_vel
  -> diff_drive_controller
  -> left/right wheel joints in Gazebo
  -> /joint_states
  -> /diff_drive_controller/odom
  -> TF odom -> base_footprint
```

Единый внешний интерфейс проекта — `/cmd_vel` типа
`geometry_msgs/msg/TwistStamped`. В ROS 2 Jazzy `diff_drive_controller` также
принимает `TwistStamped`, поэтому будущий симуляционный launch должен только
сделать remap его входа `/diff_drive_controller/cmd_vel` на `/cmd_vel`.

Интерактивный teleop запускается отдельным процессом в терминале:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Параметр `stamped:=true` заставляет внешний узел `teleop_twist_keyboard`
публиковать именно `TwistStamped` в `/cmd_vel`.

Для современного Gazebo:

```bash
ros2 run xacro xacro \
  $(ros2 pkg prefix rtk2026_description)/share/rtk2026_description/urdf/rtk2026_diff_drive_sim.urdf.xacro \
  use_meshes:=false > /tmp/rtk2026_diff_drive_sim.urdf
check_urdf /tmp/rtk2026_diff_drive_sim.urdf
```

После создания модели в Gazebo:

```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner diff_drive_controller
```

Диагностическое квантование `/joint_states` под разрешение реального энкодера:

```bash
ros2 run rtk2026_description quantize_joint_states.py --ros-args \
  -p counts_per_motor_revolution:=11.0 \
  -p quadrature_factor:=4.0 \
  -p gear_ratio:=30.0
```

`/encoder_joint_states` используется только для сравнения и тестов. Штатный
`diff_drive_controller` читает feedback непосредственно из `ros2_control`.

### Запуск симуляции со SLAM

Полный launch находится в пакете `rtk2026_bringup`:

```bash
ros2 launch rtk2026_bringup sim_slam_launch.py
```

Он запускает Gazebo Harmonic без отдельного окна, создаёт робота и тестовый
мир, поднимает `/clock`, `/scan`, `joint_state_broadcaster`,
`diff_drive_controller`, `slam_toolbox` и RViz.

Управление запускается в отдельном терминале:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true \
  -p use_sim_time:=true \
  -p frame_id:=base_footprint
```

Для запуска без RViz, например в автоматическом тесте:

```bash
ros2 launch rtk2026_bringup sim_slam_launch.py use_rviz:=false
```

## 3. Веб-камера

Макрос камеры находится в:

```text
urdf/sensors/webcam.xacro
```

Он уже включён в полную модель робота. Отдельная проверочная модель камеры:

```text
urdf/rtk2026_webcam.urdf.xacro
```

Предварительная поза относительно `base_link`:

```text
xyz = 0.125 0 0.075
rpy = 0 0 0
```

Её следует заменить измеренным положением оптического центра. Камера создаёт
`camera_link` и `camera_optical_frame`.

## 4. Параметры, требующие измерения

Для фиксированной модели:

- положение `base_link` относительно пола;
- центр масс полного корпуса;
- точные позы IMU, лидара и камеры;
- соответствие CAD-массы `wheel_link` реальной ходовой части.

Для отдельного дифференциального стенда:

- эффективный радиус колеса или ведущей звёздочки;
- эффективное расстояние между левым и правым бортом;
- масса и инерция каждой приводной стороны;
- максимальная скорость и момент после редуктора;
- число отсчётов энкодера и передаточное отношение.

## 5. STL-файлы

STL-файлы модели находятся в `meshes/tank/`:

```text
base_link.stl
wheel_link.stl
imu_link.stl
lidar_link.stl
```

Для проверки упрощённой геометрии без STL используйте `use_meshes:=false`.
