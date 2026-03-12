# Copyright 2025 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# Run WheeledLab Visual scene: MuSHR robot + camera + traversability terrain (semantic navigation).
# Keyboard: W/S throttle, A/D steer. Sensor readings printed to console periodically.
# Requires: WheeledLab cloned next to RTK folder (e.g. workspace/WheeledLab, workspace/RTK/RTK2026), wheeledlab_assets data (e.g. git lfs pull), Isaac Lab.
# Optional: pip install pynput (for keyboard control).
#
# Run from Isaac Lab root: isaaclab.bat -p path/to/run_wheeledlab_visual_scene.py
# Or: scripts/run_wheeledlab_visual.ps1 from RTK2026 root.

import argparse
import os
import sys
import threading

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="WheeledLab Visual scene: MuSHR + camera + traversability.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments (1 for single robot view).")
parser.add_argument("--no_keyboard", action="store_true", help="Disable keyboard control (robot stands still).")
parser.add_argument("--sensor_interval", type=float, default=0.5, help="Print sensor readings every N seconds.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Add WheeledLab to path: wheeledlab_tasks, wheeledlab (envs.mdp), wheeledlab_assets.
# Layout: source/{wheeledlab_tasks,wheeledlab,wheeledlab_assets}/<package_name>/
_wl_source = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "WheeledLab", "source")
_wl_tasks_root = os.path.join(_wl_source, "wheeledlab_tasks")
_wl_core_root = os.path.join(_wl_source, "wheeledlab")
_wl_assets_root = os.path.join(_wl_source, "wheeledlab_assets")
if not os.path.isdir(_wl_tasks_root):
    print("[RTK2026] WheeledLab tasks not found at %s. Clone WheeledLab and ensure path is correct." % _wl_tasks_root)
    simulation_app.close()
    sys.exit(1)
for _p in (_wl_tasks_root, _wl_core_root, _wl_assets_root, _wl_source):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Ensure wheeledlab_assets data dir exists for terrain USD output
_wl_data = os.path.join(_wl_source, "wheeledlab_assets", "data")
os.makedirs(os.path.join(_wl_data, "rgb_maps"), exist_ok=True)

# Keyboard state (thread-safe): set of currently pressed key names (lowercase).
_pressed_keys = set()
_pressed_keys_lock = threading.Lock()

def _on_key_press(key):
    try:
        k = key.char.lower() if hasattr(key, "char") and key.char else None
        if k is not None:
            with _pressed_keys_lock:
                _pressed_keys.add(k)
    except Exception:
        pass

def _on_key_release(key):
    try:
        k = key.char.lower() if hasattr(key, "char") and key.char else None
        if k is not None:
            with _pressed_keys_lock:
                _pressed_keys.discard(k)
    except Exception:
        pass

def _get_pressed_keys():
    with _pressed_keys_lock:
        return set(_pressed_keys)

def main():
    import gymnasium as gym
    import torch

    try:
        import wheeledlab_tasks  # registers Isaac-MushrVisualRL-v0
        from wheeledlab_tasks.visual.mushr_visual_env_cfg import MushrVisualPlayEnvCfg
    except Exception as e:
        print("[RTK2026] Failed to import wheeledlab_tasks: %s. Install: pip install -e wheeledlab_tasks (from WheeledLab/source)." % e)
        return

    use_keyboard = not args_cli.no_keyboard
    if use_keyboard:
        try:
            from pynput import keyboard
            listener = keyboard.Listener(on_press=_on_key_press, on_release=_on_key_release)
            listener.daemon = True
            listener.start()
        except ImportError:
            use_keyboard = False
            print("[RTK2026] Keyboard control disabled: install pynput (pip install pynput).")

    env_cfg = MushrVisualPlayEnvCfg(num_envs=args_cli.num_envs, env_spacing=0.0)
    print("[RTK2026] Creating Isaac-MushrVisualRL-v0 (MuSHR + camera + traversability terrain).")
    env = gym.make("Isaac-MushrVisualRL-v0", cfg=env_cfg)
    env.reset()
    unwrapped = env.unwrapped
    device = unwrapped.device
    action_dim = unwrapped.action_manager.total_action_dim
    robot_prim = "/World/envs/env_0/Robot" if args_cli.num_envs == 1 else "/World/envs/env_0/Robot (and env_1, ...)"
    print("[RTK2026] Scene ready. Press Play in the simulator. Robot has camera; terrain has traversability (semantic).")
    print("[RTK2026] To find the robot in viewport: Stage panel -> select '%s' -> press F (Frame)." % robot_prim)
    if use_keyboard:
        print("[RTK2026] Keyboard: W/S - gas/brake, A/D - steer left/right. Sensor readings below every %.1f s." % args_cli.sensor_interval)
    else:
        print("[RTK2026] Sensor readings below every %.1f s." % args_cli.sensor_interval)

    throttle_val = 0.6
    steer_val = 0.5
    step_interval = max(1, int(args_cli.sensor_interval / 0.02))
    step_count = 0
    while simulation_app.is_running():
        if use_keyboard:
            keys = _get_pressed_keys()
            throttle = 0.0
            steer = 0.0
            if "w" in keys:
                throttle += throttle_val
            if "s" in keys:
                throttle -= throttle_val
            if "a" in keys:
                steer += steer_val
            if "d" in keys:
                steer -= steer_val
            throttle = max(-1.0, min(1.0, throttle))
            steer = max(-1.0, min(1.0, steer))
            action = torch.tensor([[throttle, steer]], device=device, dtype=torch.float32).expand(args_cli.num_envs, action_dim)
        else:
            action = torch.zeros(args_cli.num_envs, action_dim, device=device)

        obs, _, _, _, _ = env.step(action)
        step_count += 1

        if step_count == 1:
            pos = unwrapped.scene["robot"].data.root_pos_w[0].cpu().tolist()
            print("[RTK2026] Robot position (after first step): x=%.1f y=%.1f z=%.1f" % (pos[0], pos[1], pos[2]))

        if step_count % step_interval == 0:
            robot = unwrapped.scene["robot"]
            pos = robot.data.root_pos_w[0].cpu().tolist()
            lin_vel = robot.data.root_lin_vel_w[0].cpu().tolist()
            ang_vel = robot.data.root_ang_vel_w[0].cpu().tolist()
            print("[sensors] pos x=%.2f y=%.2f z=%.2f | lin_vel vx=%.2f vy=%.2f vz=%.2f | ang_vel wx=%.2f wy=%.2f wz=%.2f | action throttle=%.2f steer=%.2f" % (
                pos[0], pos[1], pos[2],
                lin_vel[0], lin_vel[1], lin_vel[2],
                ang_vel[0], ang_vel[1], ang_vel[2],
                action[0, 0].item(), action[0, 1].item() if action_dim > 1 else 0.0,
            ))

    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
