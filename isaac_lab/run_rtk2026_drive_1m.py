# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# Drive RTK2026 diff-drive "1 m" by encoder odometry (discrete ticks), then stop.
# By default kinematic_base=True (Gazebo-like): base pose/vel from odom and cmd_vel, no physics tip-over.
# Encoder: discrete ticks (MT6701); odom from tick deltas (base_controller formulas). IMU: gyro + accel body frame.
#
# Run from Isaac Lab root: isaaclab.bat -p path/to/run_rtk2026_drive_1m.py [--headless]
# Or: scripts/run_isaac_lab.ps1 then pass -p path/to/run_rtk2026_drive_1m.py

import argparse
import csv
import math
import os

import torch
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="RTK2026 drive 1 m and log odometry.")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--robot", type=str, default="rtk2026", choices=("rtk2026", "jackal"),
                    help="Robot: rtk2026 (URDF, project config) or jackal (Clearpath USD).")
parser.add_argument("--target_m", type=float, default=1.0, help="Target distance in meters.")
parser.add_argument("--speed_ms", type=float, default=0.2, help="Forward speed m/s.")
parser.add_argument("--out_csv", type=str, default="", help="Output CSV path (default: rtk2026_odom_log_<timestamp>.csv in script dir).")
parser.add_argument("--ground", type=str, default="default", choices=("default", "plywood"),
                    help="Ground surface: default (grid) or plywood (fanera) for lab-like floor.")
parser.add_argument("--kinematic_base", action="store_true", default=True,
                    help="Gazebo-like: base pose/vel from odom and cmd_vel, no physics tip-over (default: True).")
parser.add_argument("--no_kinematic_base", action="store_false", dest="kinematic_base",
                    help="Use full physics for base (can tip).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab.utils.math import euler_xyz_from_quat, quat_apply_inverse, quat_from_euler_xyz


def _urdf_path(name):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), name)


def _load_rtk2026_base_config():
    """Load wheel_separation and ticks_per_meter from rtk2026_base config (project params)."""
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    yaml_path = os.path.join(repo_root, "src", "rtk2026_base", "config", "base_controller.yaml")
    wheel_separation = WHEEL_SEPARATION_RTK2026_M
    ticks_per_meter = 1000.0
    if os.path.isfile(yaml_path):
        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("wheel_separation:"):
                        wheel_separation = float(line.split(":", 1)[1].strip())
                    elif line.startswith("ticks_per_meter:"):
                        ticks_per_meter = float(line.split(":", 1)[1].strip())
        except Exception:
            pass
    return wheel_separation, ticks_per_meter


# RTK2026: match rtk2026_diff_drive.urdf. Real robot (xacro) uses track radius 0.026.
WHEEL_RADIUS_RTK2026_M = 0.06
WHEEL_SEPARATION_RTK2026_M = 0.25

# Clearpath Jackal: 194 mm diameter -> radius 0.097 m (Isaac Sim / Clearpath docs)
WHEEL_RADIUS_JACKAL_M = 0.097
WHEEL_SEPARATION_JACKAL_M = 0.366  # Jackal spec ~366 mm


@configclass
class RTK2026Drive1mSceneCfg(InteractiveSceneCfg):
    """Scene: ground, light, RTK2026 diff-drive as Articulation for velocity control and logging."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = ArticulationCfg(
        prim_path="/World/RTK2026",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=_urdf_path("rtk2026_diff_drive.urdf"),
            fix_base=False,
            replace_cylinders_with_capsules=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_linear_velocity=1000.0,
                max_angular_velocity=100000.0,
                max_depenetration_velocity=100.0,
                enable_gyroscopic_forces=True,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
            ),
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                    stiffness=50.0,
                    damping=5.0,
                ),
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.06),
            joint_pos={"left_wheel_joint": 0.0, "right_wheel_joint": 0.0},
        ),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["left_wheel_joint", "right_wheel_joint"],
                effort_limit_sim=10.0,
                velocity_limit_sim=50.0,
                stiffness=0.0,
                damping=0.5,
            ),
        },
    )


def _make_jackal_scene_cfg():
    from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
    return type("JackalDrive1mSceneCfg", (InteractiveSceneCfg,), {
        "ground": AssetBaseCfg(
            prim_path="/World/defaultGroundPlane",
            spawn=sim_utils.GroundPlaneCfg(),
        ),
        "light": AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
        ),
        "robot": ArticulationCfg(
            prim_path="/World/Jackal",
            spawn=sim_utils.UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Clearpath/Jackal/jackal.usd",
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    rigid_body_enabled=True,
                    max_linear_velocity=1000.0,
                    max_angular_velocity=100000.0,
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, 0.0),
                joint_pos={".*": 0.0},
                joint_vel={".*": 0.0},
            ),
            actuators={
                "wheels": ImplicitActuatorCfg(
                    joint_names_expr=[".*wheel.*"],
                    effort_limit_sim=100.0,
                    velocity_limit_sim=50.0,
                    stiffness=0.0,
                    damping=1.0,
                ),
            },
        ),
    })


def _find_wheel_joint_indices(robot):
    """Return (left_id, right_id) for 2-wheel, or (wheel_ids, None) for 4-wheel (all same velocity).
    Jackal USD has front_left, front_right, rear_left, rear_right.
    """
    names = robot.joint_names
    wheel_indices = [i for i, n in enumerate(names) if "wheel" in n.lower()]
    if not wheel_indices:
        raise RuntimeError("No wheel joints found. Joint names: %s" % names)
    # 2-wheel robot: left and right
    if len(wheel_indices) == 2:
        n0, n1 = names[wheel_indices[0]].lower(), names[wheel_indices[1]].lower()
        if "left" in n0 and "right" in n1:
            return wheel_indices[0], wheel_indices[1], 2
        if "left" in n1 and "right" in n0:
            return wheel_indices[1], wheel_indices[0], 2
        return wheel_indices[0], wheel_indices[1], 2
    # 4-wheel (Jackal): all get same omega for straight drive
    return wheel_indices, None, 4


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    if args_cli.robot == "jackal":
        JackalSceneCfg = _make_jackal_scene_cfg()
        scene_cfg = JackalSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        wheel_radius = WHEEL_RADIUS_JACKAL_M
        wheel_separation = WHEEL_SEPARATION_JACKAL_M
        ticks_per_m = 1000.0
        robot_prim = "/World/Jackal"
    else:
        scene_cfg = RTK2026Drive1mSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        wheel_radius = WHEEL_RADIUS_RTK2026_M
        wheel_separation, ticks_per_m = _load_rtk2026_base_config()
        robot_prim = "/World/RTK2026"

    if args_cli.ground == "plywood":
        from isaaclab.sim.spawners import materials
        plywood_color = (0.65, 0.5, 0.35)
        plywood_physics = materials.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.45)
        scene_cfg.ground = AssetBaseCfg(
            prim_path="/World/defaultGroundPlane",
            spawn=sim_utils.GroundPlaneCfg(
                color=plywood_color,
                size=(20.0, 20.0),
                physics_material=plywood_physics,
            ),
        )
        print("[Drive1m] Ground: plywood (fanera)")

    scene = InteractiveScene(scene_cfg)
    sim.reset()
    sim.set_camera_view(eye=(4.0, 0.0, 2.0), target=(1.0, 0.0, 0.2))
    sim.render()

    robot = scene.articulations["robot"]
    n_wheels = 2
    if args_cli.robot == "rtk2026":
        # find_joints returns (joint_indices, joint_names)
        left_ids, _ = robot.find_joints("left_wheel_joint")
        right_ids, _ = robot.find_joints("right_wheel_joint")
        left_id = int(left_ids[0]) if left_ids else 0
        right_id = int(right_ids[0]) if right_ids else 1
        wheel_ids = [left_id, right_id]
    else:
        res = _find_wheel_joint_indices(robot)
        n_wheels = res[2]
        if n_wheels == 2:
            left_id, right_id = res[0], res[1]
            wheel_ids = [left_id, right_id]
        else:
            wheel_ids = res[0]
            left_id, right_id = wheel_ids[0], wheel_ids[1]
    print("[Drive1m] Robot=%s n_wheels=%s wheel_ids=%s (names: %s)" % (
        args_cli.robot, n_wheels, wheel_ids,
        [robot.joint_names[i] for i in wheel_ids]))

    device = robot.device
    # Forward speed -> wheel angular velocity (rad/s). v = omega * r => omega = v / r
    omega = args_cli.speed_ms / wheel_radius
    target_m = args_cli.target_m
    dt = sim.get_physics_dt()

    log_rows = []
    step = 0
    max_steps = int(60.0 / dt)  # ~60 s max

    # Encoder/odom: params from project (rtk2026_base). Unwrap joint angles in case sim wraps at 2*pi.
    meters_per_tick = 1.0 / ticks_per_m
    total_odom_dist = 0.0
    prev_left_count = None
    prev_right_count = None
    left_unwrapped, right_unwrapped = 0.0, 0.0
    prev_left_pos, prev_right_pos = None, None
    odom_x, odom_y, odom_theta = 0.0, 0.0, 0.0  # encoder-based pose (base_controller logic)
    prev_lin_vel_b = None  # for IMU accel
    GRAVITY_W = torch.tensor([0.0, 0.0, -9.81], device=device, dtype=torch.float32)
    vel_stop_steps = int(0.5 / dt)

    print("[Drive1m] target=%.2f m (by encoder odom), speed=%.2f m/s, wheel_radius=%.3f m, ticks_per_m=%.0f" % (
        target_m, args_cli.speed_ms, wheel_radius, ticks_per_m))
    print("[Drive1m] Encoder: discrete ticks (MT6701). IMU: gyro + accel body frame. Stop when odom_dist >= target.")

    vel_forward = torch.tensor([[omega] * n_wheels], device=device, dtype=robot.data.joint_vel_target.dtype)
    vel_zero = torch.zeros_like(vel_forward)
    stopped_by_odom = False
    stop_countdown = -1  # after stop: number of steps left with vel=0

    # Kinematic base (Gazebo-like): base pose from odom, no tip-over. Need initial z.
    kinematic_base = getattr(args_cli, "kinematic_base", True)
    init_z = 0.06
    if kinematic_base:
        scene.update(dt)
        init_z = robot.data.root_pos_w[0, 2].item()
        print("[Drive1m] Kinematic base: pose/vel from odom and cmd_vel (no physics tip-over).")

    while simulation_app.is_running() and step < max_steps:
        # Decide velocity: drive until odometry says we've gone target_m, then stop
        if stop_countdown > 0:
            vel_target = vel_zero
            stop_countdown -= 1
        elif stop_countdown == 0:
            break
        else:
            vel_target = vel_forward

        # Gazebo-like: set base pose from odom and velocity from cmd so base does not tip
        if kinematic_base:
            roll = torch.tensor([0.0], device=device)
            pitch = torch.tensor([0.0], device=device)
            yaw_t = torch.tensor([odom_theta], device=device)
            quat = quat_from_euler_xyz(roll, pitch, yaw_t)  # (1, 4) wxyz
            root_pose = torch.zeros(1, 7, device=device)
            root_pose[0, 0] = odom_x
            root_pose[0, 1] = odom_y
            root_pose[0, 2] = init_z
            root_pose[0, 3:7] = quat[0]
            robot.write_root_pose_to_sim(root_pose, env_ids=[0])
            vx = args_cli.speed_ms * math.cos(odom_theta) if stop_countdown < 0 else 0.0
            vy = args_cli.speed_ms * math.sin(odom_theta) if stop_countdown < 0 else 0.0
            root_vel = torch.zeros(1, 6, device=device)
            root_vel[0, 0], root_vel[0, 1], root_vel[0, 2] = vx, vy, 0.0
            root_vel[0, 3], root_vel[0, 4], root_vel[0, 5] = 0.0, 0.0, 0.0
            robot.write_root_velocity_to_sim(root_vel, env_ids=[0])

        robot.set_joint_velocity_target(vel_target, joint_ids=wheel_ids, env_ids=None)
        scene.write_data_to_sim()
        sim.step(render=True)
        scene.update(dt)

        t = step * dt
        pos_w = robot.data.root_pos_w[0]
        quat_w = robot.data.root_quat_w[0:1]
        _roll, _pitch, yaw_t = euler_xyz_from_quat(quat_w)
        yaw = yaw_t[0].item()
        x, y = pos_w[0].item(), pos_w[1].item()
        jpos = robot.data.joint_pos[0]
        left_pos = jpos[left_id].item()
        right_pos = jpos[right_id].item()
        # Unwrap angles (sim may report [-pi, pi]); then encoder ticks from unwrapped travel
        if prev_left_pos is not None:
            dl = left_pos - prev_left_pos
            if dl > math.pi:
                dl -= 2.0 * math.pi
            elif dl < -math.pi:
                dl += 2.0 * math.pi
            left_unwrapped += dl
            dr = right_pos - prev_right_pos
            if dr > math.pi:
                dr -= 2.0 * math.pi
            elif dr < -math.pi:
                dr += 2.0 * math.pi
            right_unwrapped += dr
        else:
            left_unwrapped = left_pos
            right_unwrapped = right_pos
        prev_left_pos = left_pos
        prev_right_pos = right_pos
        left_count = int(round(left_unwrapped * wheel_radius * ticks_per_m))
        right_count = int(round(right_unwrapped * wheel_radius * ticks_per_m))

        # Odometry from tick deltas (same as base_controller_node)
        if prev_left_count is not None:
            d_left_m = (left_count - prev_left_count) * meters_per_tick
            d_right_m = (right_count - prev_right_count) * meters_per_tick
            d_center = (d_left_m + d_right_m) * 0.5
            d_theta = (d_right_m - d_left_m) / wheel_separation
            total_odom_dist += d_center
            odom_theta += d_theta
            odom_x += d_center * math.cos(odom_theta)
            odom_y += d_center * math.sin(odom_theta)
        prev_left_count = left_count
        prev_right_count = right_count

        # IMU (body frame): gyro = ang_vel_b, accel = (lin_vel_b - prev)/dt + gravity_b
        lin_vel_b = robot.data.root_lin_vel_b[0]
        ang_vel_b = robot.data.root_ang_vel_b[0]
        quat = robot.data.root_quat_w[0]
        gravity_b = quat_apply_inverse(quat.unsqueeze(0), GRAVITY_W.unsqueeze(0)).squeeze(0)
        if prev_lin_vel_b is not None:
            accel_b = (lin_vel_b - prev_lin_vel_b) / dt + gravity_b
        else:
            accel_b = gravity_b.clone()
        prev_lin_vel_b = lin_vel_b.clone()

        log_rows.append({
            "sim_time": t,
            "gt_x": x,
            "gt_y": y,
            "gt_yaw": yaw,
            "odom_dist_cum": total_odom_dist,
            "odom_x": odom_x,
            "odom_y": odom_y,
            "odom_yaw": odom_theta,
            "left_count": left_count,
            "right_count": right_count,
            "left_wheel_pos": left_pos,
            "right_wheel_pos": right_pos,
            "imu_gyro_x": ang_vel_b[0].item(),
            "imu_gyro_y": ang_vel_b[1].item(),
            "imu_gyro_z": ang_vel_b[2].item(),
            "imu_accel_x": accel_b[0].item(),
            "imu_accel_y": accel_b[1].item(),
            "imu_accel_z": accel_b[2].item(),
        })

        if not stopped_by_odom and total_odom_dist >= target_m:
            stopped_by_odom = True
            stop_countdown = vel_stop_steps
            print("[Drive1m] Odometry reached %.2f m at step %d (t=%.2f s). Stopping (GT x=%.3f m)." % (target_m, step, t, x))
        step += 1

    if not stopped_by_odom and step >= max_steps:
        print("[Drive1m] Timeout at step %d (odom=%.3f m, gt_x=%.3f m)." % (
            step, total_odom_dist, log_rows[-1]["gt_x"] if log_rows else 0))

    # Summary: encoder odom vs GT
    if log_rows:
        last = log_rows[-1]
        gt_dist = (last["gt_x"] ** 2 + last["gt_y"] ** 2) ** 0.5
        odom_final = last["odom_dist_cum"]
        odom_dist_xy = (last["odom_x"] ** 2 + last["odom_y"] ** 2) ** 0.5
        print("[Drive1m] Result: odom_dist_cum=%.4f m, odom_pose norm=%.4f m, GT distance=%.4f m, error(GT-target)=%.4f m" % (
            odom_final, odom_dist_xy, gt_dist, gt_dist - target_m))
        print("[Drive1m] Encoder pose (odom_x, odom_y, odom_yaw) vs GT (gt_x, gt_y, gt_yaw): (%.3f, %.3f, %.3f) vs (%.3f, %.3f, %.3f)" % (
            last["odom_x"], last["odom_y"], last["odom_yaw"], last["gt_x"], last["gt_y"], last["gt_yaw"]))

    # Save CSV
    out_path = args_cli.out_csv.strip()
    if not out_path:
        import time
        out_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "rtk2026_odom_log_%s.csv" % time.strftime("%Y%m%d_%H%M%S"),
        )
    fieldnames = [
        "sim_time", "gt_x", "gt_y", "gt_yaw",
        "odom_dist_cum", "odom_x", "odom_y", "odom_yaw",
        "left_count", "right_count", "left_wheel_pos", "right_wheel_pos",
        "imu_gyro_x", "imu_gyro_y", "imu_gyro_z",
        "imu_accel_x", "imu_accel_y", "imu_accel_z",
    ]
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(log_rows)
    print("[Drive1m] Saved %d rows to %s" % (len(log_rows), out_path))

    # Keep window open if not headless
    while simulation_app.is_running() and not args_cli.headless:
        sim.step(render=True)
        scene.update(dt)


if __name__ == "__main__":
    main()
    simulation_app.close()
