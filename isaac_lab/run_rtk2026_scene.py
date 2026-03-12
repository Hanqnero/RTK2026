# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# Scene for RTK2026 in Isaac Lab: ground, light, robot (URDF), optional ROS2 /clock.
# Pattern from WheeledLab: enable_cameras=True so the rendering kit and viewport load.
#
# Run: from Isaac Lab root: isaaclab.bat -p path/to/run_rtk2026_scene.py --num_envs 1
# Or: scripts/run_isaac_lab.ps1 from RTK2026 repo root.
# Or from conda env with Isaac Lab in path: python run_rtk2026_scene.py (no --headless).
#
# ROS2: set ROS_DOMAIN_ID=0. After Play, /clock is published if bridge is available.

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="RTK2026 scene in Isaac Lab.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--ros_domain_id", type=int, default=0, help="ROS2 domain ID for bridge.")
parser.add_argument("--no_robot", action="store_true", help="Only ground and light (WheeledLab-style minimal scene, no URDF/mesh warnings).")
parser.add_argument("--ground", type=str, default="default", choices=("default", "plywood"),
                    help="Ground surface: default (grid) or plywood (fanera).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# WheeledLab-style: enable cameras so experience file has viewport (isaaclab.python.rendering.kit)
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

# Try to load WheeledLab MuSHR (stable USD robot). If data is missing, we fall back to our URDF with fix_base.
WHEELEDLAB_ROBOT_CFG = None
_wheeledlab_source = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "WheeledLab", "source")
if os.path.isdir(_wheeledlab_source) and _wheeledlab_source not in sys.path:
    sys.path.insert(0, _wheeledlab_source)
try:
    from wheeledlab_assets.mushr import MUSHR_SUS_CFG
    _mushr_usd = MUSHR_SUS_CFG.spawn.usd_path
    if os.path.isfile(_mushr_usd):
        WHEELEDLAB_ROBOT_CFG = MUSHR_SUS_CFG.replace(
            prim_path="/World/RTK2026",
            init_state=MUSHR_SUS_CFG.init_state.replace(pos=(0.0, 0.0, 1.05)),
        )
except Exception:
    pass

# Optional: ROS2 bridge (Isaac Sim extension). Set to None if creation fails.
_ros2_clock_impulse_attr = None
_ros2_cmdvel_impulse_attr = None

# Graph path and node names for ROS2 /clock and /odom (unique to avoid conflicts)
_ROS2_GRAPH_PATH = "/World/RTK2026_ROS2Clock"
_NODE_READ_SIM_TIME = "ReadSimTime"
_NODE_CONTEXT = "ROS2Context"
_NODE_PUBLISH_CLOCK = "PublishClock"
_NODE_IMPULSE = "OnImpulse"
# Odometry: chassis prim -> Compute Odometry -> ROS2 Publish Odometry
_NODE_COMPUTE_ODOM = "ComputeOdometry"
_NODE_PUBLISH_ODOM = "PublishOdometry"
# Default robot prim path (used when scene has robot)
_DEFAULT_ROBOT_PRIM_PATH = "/World/RTK2026"


def _list_ros2_bridge_extensions():
    """List extensions related to ROS2/bridge so user can see why bridge may be missing. Returns list of (id, available, enabled)."""
    out = []
    try:
        import omni.kit.app
        mgr = omni.kit.app.get_app().get_extension_manager()
        for ext_id in mgr.get_loaded_extensions() + list(mgr.get_available_extensions()):
            if "ros2" in ext_id.lower() or "bridge" in ext_id.lower():
                try:
                    avail = mgr.is_extension_available(ext_id)
                    en = mgr.is_extension_enabled(ext_id) if avail else False
                    out.append((ext_id, avail, en))
                except Exception:
                    out.append((ext_id, False, False))
    except Exception:
        pass
    return out


def _enable_ros2_bridge_extension():
    """Enable Isaac Sim ROS2 bridge extension so OmniGraph nodes are available. Returns True if enabled."""
    try:
        import omni.kit.app
        mgr = omni.kit.app.get_app().get_extension_manager()
        for ext_id in ("omni.isaac.ros2_bridge", "isaacsim.ros2.bridge"):
            if mgr.is_extension_available(ext_id):
                mgr.set_extension_enabled(ext_id, True)
                return True
        return False
    except Exception:
        return False


def _wait_for_extension_updates(count: int = 3):
    """Run a few app updates so extension loading can complete."""
    try:
        import omni.kit.app
        app = omni.kit.app.get_app()
        for _ in range(count):
            app.update()
    except Exception:
        pass


def _setup_ros2_clock(domain_id: int, robot_prim_path: str | None = None):
    """Create OmniGraph with ROS2 Context, PublishClock, and optionally PublishOdometry (if robot_prim_path set).
    Returns (impulse_attr, odom_ok) where odom_ok is True if /odom graph was added."""
    if not _enable_ros2_bridge_extension():
        return None, False
    _wait_for_extension_updates(5)
    try:
        import omni.graph.core as og
    except ImportError:
        return None, False
    keys = og.Controller.Keys
    # Node type names: Isaac Sim 5.x uses omni.isaac.*, 4.x uses isaacsim.*
    variants = [
        (
            "omni.isaac.core_nodes.IsaacReadSimulationTime",
            "omni.isaac.ros2_bridge.ROS2Context",
            "omni.isaac.ros2_bridge.ROS2PublishClock",
            "omni.isaac.core_nodes.IsaacComputeOdometry",
            "omni.isaac.ros2_bridge.ROS2PublishOdometry",
        ),
        (
            "isaacsim.core.nodes.IsaacReadSimulationTime",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.ros2.bridge.ROS2PublishClock",
            "isaacsim.core.nodes.IsaacComputeOdometry",
            "isaacsim.ros2.bridge.ROS2PublishOdometry",
        ),
    ]
    for read_type, context_type, clock_type, compute_odom_type, publish_odom_type in variants:
        try:
            create_nodes = [
                (_NODE_READ_SIM_TIME, read_type),
                (_NODE_CONTEXT, context_type),
                (_NODE_PUBLISH_CLOCK, clock_type),
                (_NODE_IMPULSE, "omni.graph.action.OnImpulseEvent"),
            ]
            connect = [
                (f"{_NODE_IMPULSE}.outputs:execOut", f"{_NODE_PUBLISH_CLOCK}.inputs:execIn"),
                (f"{_NODE_READ_SIM_TIME}.outputs:simulationTime", f"{_NODE_PUBLISH_CLOCK}.inputs:timeStamp"),
                (f"{_NODE_CONTEXT}.outputs:context", f"{_NODE_PUBLISH_CLOCK}.inputs:context"),
            ]
            set_values = [
                (f"{_NODE_PUBLISH_CLOCK}.inputs:topicName", "/clock"),
                (f"{_NODE_CONTEXT}.inputs:domain_id", domain_id),
                (f"{_NODE_CONTEXT}.inputs:useDomainIDEnvVar", False),
            ]
            if robot_prim_path:
                create_nodes.extend([
                    (_NODE_COMPUTE_ODOM, compute_odom_type),
                    (_NODE_PUBLISH_ODOM, publish_odom_type),
                ])
                connect.extend([
                    (f"{_NODE_IMPULSE}.outputs:execOut", f"{_NODE_COMPUTE_ODOM}.inputs:execIn"),
                    (f"{_NODE_COMPUTE_ODOM}.outputs:execOut", f"{_NODE_PUBLISH_ODOM}.inputs:execIn"),
                    (f"{_NODE_COMPUTE_ODOM}.outputs:position", f"{_NODE_PUBLISH_ODOM}.inputs:position"),
                    (f"{_NODE_COMPUTE_ODOM}.outputs:orientation", f"{_NODE_PUBLISH_ODOM}.inputs:orientation"),
                    (f"{_NODE_COMPUTE_ODOM}.outputs:linearVelocity", f"{_NODE_PUBLISH_ODOM}.inputs:linearVelocity"),
                    (f"{_NODE_COMPUTE_ODOM}.outputs:angularVelocity", f"{_NODE_PUBLISH_ODOM}.inputs:angularVelocity"),
                    (f"{_NODE_READ_SIM_TIME}.outputs:simulationTime", f"{_NODE_PUBLISH_ODOM}.inputs:timeStamp"),
                    (f"{_NODE_CONTEXT}.outputs:context", f"{_NODE_PUBLISH_ODOM}.inputs:context"),
                ])
                set_values.extend([
                    (f"{_NODE_COMPUTE_ODOM}.inputs:chassisPrim", robot_prim_path),
                    (f"{_NODE_PUBLISH_ODOM}.inputs:topicName", "/odom"),
                    (f"{_NODE_PUBLISH_ODOM}.inputs:chassisFrameId", "base_link"),
                    (f"{_NODE_PUBLISH_ODOM}.inputs:odomFrameId", "odom"),
                ])
            og.Controller.edit(
                {"graph_path": _ROS2_GRAPH_PATH, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: create_nodes,
                    keys.CONNECT: connect,
                    keys.SET_VALUES: set_values,
                },
            )
            impulse_attr = og.Controller.attribute(f"{_ROS2_GRAPH_PATH}/{_NODE_IMPULSE}.state:enableImpulse")
            return impulse_attr, bool(robot_prim_path)
        except Exception:
            continue
    return None, False


# cmd_vel graph: Subscribe Twist -> Differential Controller -> Articulation Controller (optional, needs wheeled_robots)
_ROS2_CMDVEL_GRAPH_PATH = "/World/RTK2026_ROS2CmdVel"
_WHEEL_RADIUS_RTK2026 = 0.06
_WHEEL_SEPARATION_RTK2026 = 0.25
_RTK2026_WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]


def _remove_graph_prim_if_exists(graph_path: str) -> None:
    """Remove existing prim at graph_path so a new graph can be created (avoids 'graph already exists')."""
    try:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if stage:
            prim = stage.GetPrimAtPath(graph_path)
            if prim.IsValid():
                stage.RemovePrim(prim.GetPath())
    except Exception:
        pass


def _setup_ros2_cmd_vel(domain_id: int, robot_prim_path: str):
    """Create OmniGraph: ROS2 Subscribe Twist (cmd_vel) -> Differential Controller -> Articulation Controller.
    Returns impulse attribute to trigger each step, or None if graph was not created."""
    try:
        import omni.graph.core as og
    except ImportError:
        return None
    _remove_graph_prim_if_exists(_ROS2_CMDVEL_GRAPH_PATH)
    keys = og.Controller.Keys
    variants = [
        (
            "omni.isaac.ros2_bridge.ROS2Context",
            "omni.isaac.ros2_bridge.ROS2SubscribeTwist",
            "omni.graph.nodes.BreakVector3",
            "omni.isaac.wheeled_robots.DifferentialController",
            "omni.isaac.core_nodes.IsaacArticulationController",
        ),
        (
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.ros2.bridge.ROS2SubscribeTwist",
            "omni.graph.nodes.BreakVector3",
            "isaacsim.robot.wheeled_robots.DifferentialController",
            "isaacsim.core.nodes.IsaacArticulationController",
        ),
    ]
    for context_type, twist_type, break_type, diff_type, art_type in variants:
        try:
            og.Controller.edit(
                {"graph_path": _ROS2_CMDVEL_GRAPH_PATH, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("Context", context_type),
                        ("SubscribeTwist", twist_type),
                        ("BreakLinear", break_type),
                        ("BreakAngular", break_type),
                        ("Differential", diff_type),
                        ("Articulation", art_type),
                        ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ],
                    keys.CONNECT: [
                        ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                        ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                        ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                        ("BreakLinear.outputs:x", "Differential.inputs:linearVelocity"),
                        ("BreakAngular.outputs:z", "Differential.inputs:angularVelocity"),
                        ("Differential.outputs:velocityCommand", "Articulation.inputs:velocityCommand"),
                        ("OnImpulse.outputs:execOut", "Articulation.inputs:execIn"),
                    ],
                    keys.SET_VALUES: [
                        ("Context.inputs:domain_id", domain_id),
                        ("Context.inputs:useDomainIDEnvVar", False),
                        ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
                        ("Differential.inputs:wheelRadius", _WHEEL_RADIUS_RTK2026),
                        ("Differential.inputs:wheelDistance", _WHEEL_SEPARATION_RTK2026),
                        ("Articulation.inputs:robotPath", robot_prim_path),
                        ("Articulation.inputs:jointNames", _RTK2026_WHEEL_JOINTS),
                    ],
                },
            )
            return og.Controller.attribute(f"{_ROS2_CMDVEL_GRAPH_PATH}/OnImpulse.state:enableImpulse")
        except Exception:
            continue
    return None


def _urdf_path(filename: str):
    """Absolute path to a URDF file next to this script."""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)


@configclass
class RTK2026SceneMinimalCfg(InteractiveSceneCfg):
    """Minimal scene: ground and light only (like WheeledLab). No robot, no URDF importer warnings."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )


@configclass
class RTK2026SceneCfg(InteractiveSceneCfg):
    """Scene: ground, light, diff-drive robot (URDF, fix_base so it does not flip). Use WheeledLab scene when assets available."""

    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )
    robot = AssetBaseCfg(
        prim_path="/World/RTK2026",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=_urdf_path("rtk2026_diff_drive.urdf"),
            fix_base=True,
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
    )


if WHEELEDLAB_ROBOT_CFG is not None:
    _wl_robot_cfg = WHEELEDLAB_ROBOT_CFG

    @configclass
    class RTK2026SceneWheeledLabCfg(InteractiveSceneCfg):
        """Scene: ground, light, MuSHR from WheeledLab (USD)."""

        ground = AssetBaseCfg(
            prim_path="/World/defaultGroundPlane",
            spawn=sim_utils.GroundPlaneCfg(),
        )
        light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
        )
        robot = _wl_robot_cfg
else:
    RTK2026SceneWheeledLabCfg = None


def main():
    global _ros2_clock_impulse_attr
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    if args_cli.no_robot:
        scene_cfg = RTK2026SceneMinimalCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        print("[RTK2026] Minimal scene (ground + light only, no robot).")
    elif RTK2026SceneWheeledLabCfg is not None:
        scene_cfg = RTK2026SceneWheeledLabCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        print("[RTK2026] Loading scene with WheeledLab MuSHR (USD) at /World/RTK2026.")
    else:
        scene_cfg = RTK2026SceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
        print("[RTK2026] Loading scene with diff-drive robot (URDF, fix_base) at /World/RTK2026. For movable robot, add WheeledLab assets data.")
    if args_cli.ground == "plywood":
        from isaaclab.sim.spawners import materials
        scene_cfg.ground = AssetBaseCfg(
            prim_path="/World/defaultGroundPlane",
            spawn=sim_utils.GroundPlaneCfg(
                color=(0.65, 0.5, 0.35),
                size=(20.0, 20.0),
                physics_material=materials.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.45),
            ),
        )
        print("[RTK2026] Ground: plywood (fanera)")
    scene = InteractiveScene(scene_cfg)
    sim.reset()
    sim.set_camera_view(eye=(3.0, 0.0, 2.0), target=(0.0, 0.0, 0.5))
    sim.render()

    robot_prim_for_bridge = _DEFAULT_ROBOT_PRIM_PATH if not args_cli.no_robot else None
    global _ros2_clock_impulse_attr, _ros2_cmdvel_impulse_attr
    _ros2_clock_impulse_attr, _ros2_odom_ok = _setup_ros2_clock(args_cli.ros_domain_id, robot_prim_for_bridge)
    if _ros2_clock_impulse_attr is not None:
        print("[RTK2026] ROS2 /clock bridge enabled. ROS_DOMAIN_ID=%s" % args_cli.ros_domain_id)
        if _ros2_odom_ok:
            print("[RTK2026] ROS2 /odom bridge enabled (frame odom, base_link). Nav2/Docker stack can use it.")
    else:
        print("[RTK2026] ROS2 bridge not available; run Docker stack with use_fake_scan. ROS_DOMAIN_ID=%s" % args_cli.ros_domain_id)
    if robot_prim_for_bridge and _ros2_clock_impulse_attr is not None:
        _ros2_cmdvel_impulse_attr = _setup_ros2_cmd_vel(args_cli.ros_domain_id, robot_prim_for_bridge)
        if _ros2_cmdvel_impulse_attr is not None:
            print("[RTK2026] ROS2 /cmd_vel bridge enabled. Nav2 can drive the robot.")
        else:
            print("[RTK2026] ROS2 /cmd_vel bridge not available (install wheeled_robots?). Drive robot from script or UI.")
    if not args_cli.no_robot:
        if RTK2026SceneWheeledLabCfg is not None:
            print("[RTK2026] Scene ready. Press Play: MuSHR robot (WheeledLab) at /World/RTK2026.")
        else:
            print("[RTK2026] Scene ready. Press Play: diff-drive robot (URDF, fixed base) at /World/RTK2026.")
        for ext_id, avail, enabled in _list_ros2_bridge_extensions():
            print("[RTK2026]   Extension: %s  available=%s  enabled=%s" % (ext_id, avail, enabled))
        if not _list_ros2_bridge_extensions():
            print("[RTK2026]   No ROS2/bridge extensions found. Install Isaac Sim with ROS2 Bridge or use Docker stack without /clock.")

    while simulation_app.is_running():
        sim.step(render=True)
        scene.update(sim.get_physics_dt())
        try:
            import omni.graph.core as og
            if _ros2_clock_impulse_attr is not None:
                og.Controller.set(_ros2_clock_impulse_attr, True)
            if _ros2_cmdvel_impulse_attr is not None:
                og.Controller.set(_ros2_cmdvel_impulse_attr, True)
        except Exception:
            pass


if __name__ == "__main__":
    main()
    simulation_app.close()
