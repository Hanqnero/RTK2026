# RTK2026 v Isaac Lab

Simulyaciya RTK2026 v Isaac Lab s publikaciej /odom, /scan i podpiskoj na /cmd_vel. ROS2 stek zapuskaetsa otdelno s `use_sim_time:=true`.

## Trebovaniya

- Isaac Sim (backend dlya Isaac Lab)
- Isaac Lab ustanovlen i skonfigurirovan
- ROS2 Humble (v drugoj sessii ili na drugoj mashine po seti)

## Topiki i TF

Isaac Lab (ili most Isaac Sim ROS2) dolzhen:

- **Publikovaty:** `/odom` (nav_msgs/Odometry), tf `odom` -> `base_link`
- **Publikovaty:** `/scan` (sensor_msgs/LaserScan) esli nuzhen SLAM/Nav2
- **Podpisyvatsya:** `/cmd_vel` (geometry_msgs/Twist) — linejnaya i uglovaya skorost bazy.

So storony ROS2 zapuskaetsa launch s vremenem simulyacii, bez drajvera i base_controller. Esli sim ne daet /scan (LaserScan), dobavit' `use_fake_scan:=true`.

```bash
source install/setup.bash
ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true
ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_slam:=true
```

S SLAM i Nav2:

```bash
ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_slam:=true use_navigation:=true
```

V simulyacii odometriyu i lidar daet Isaac; robot_state_publisher publikuet derevo iz URDF (base_link -> lidar_link, camera_link, imu_link).

## Zapusk sceny v Isaac Lab

1. Aktivirovat okruzhenie Isaac Lab (conda/env). Isaac Lab dolzhen byt v kataloge-urovne s RTK (napr. workspace/IsaacLab, workspace/RTK/RTK2026) ili ukazat put parametrom -IsaacLabPath.
2. Iz kornya RTK2026: Windows — `.\scripts\run_isaac_lab.ps1`; Linux/WSL — `./scripts/run_isaac_lab.sh`. Zapuskaetsya scena `isaac_lab/run_rtk2026_scene.py` (pol, svet; pri dostupnosti ROS2 mosta — `/clock`).
3. V okne simulyatora nazhat **Play**.
4. Vo vtorom terminale zapustit ROS2 stek (Docker): `.\scripts\run_docker_simulation.ps1` ili lokalno `ros2 launch rtk2026_simulation simulation.launch.py use_sim_time:=true use_fake_scan:=true use_slam:=true`.

Dlya polnogo mosta (/odom, /cmd_vel) — dobavit uzly v Isaac Sim (Action Graph) ili sm. oficialnuyu dokumentaciyu Isaac Sim / Isaac Lab.

## Parametry robota (sovmestimost s URDF)

- `base_link` — osnovnaya ramka
- `wheel_separation` 0.25 m (po rtk2026_base, shirina 250 mm)
- Dlya Nav2/SLAM ozhidaetsa `base_link`, `odom`, `map`, topic `/scan`
