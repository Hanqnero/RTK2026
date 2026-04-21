#!/usr/bin/env python3
"""Монитор качества v3: robot_xy (TF) vs последняя цель nav2_chain_v3."""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import time
from datetime import datetime, timezone


TARGET_RE = re.compile(
    r"nav2_chain_v3 .*?points=\[.*?\((-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)\)\]"
)
TF_RE = re.compile(r"- Translation:\s*\[\s*(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?),")


def run(cmd: list[str]) -> str:
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return (proc.stdout or "") + (proc.stderr or "")


def read_latest_target(compose_file: str, since_window: str) -> tuple[float, float] | None:
    out = run(
        [
            "docker",
            "compose",
            "-f",
            compose_file,
            "logs",
            "route_editor_full",
            "--since",
            since_window,
        ]
    )
    matches = list(TARGET_RE.finditer(out))
    if not matches:
        return None
    m = matches[-1]
    return (float(m.group(1)), float(m.group(2)))


def read_robot_xy(compose_file: str) -> tuple[float, float] | None:
    out = run(
        [
            "docker",
            "compose",
            "-f",
            compose_file,
            "exec",
            "odometry",
            "bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo map base_footprint",
        ]
    )
    matches = list(TF_RE.finditer(out))
    if not matches:
        return None
    m = matches[-1]
    return (float(m.group(1)), float(m.group(2)))


def main() -> int:
    parser = argparse.ArgumentParser(description="v3 odom monitor")
    parser.add_argument("--route-compose", default="docker-compose.route_editor.full.yml")
    parser.add_argument("--sim-compose", default="docker-compose.sim.yml")
    parser.add_argument("--interval-sec", type=float, default=2.0)
    parser.add_argument("--loops", type=int, default=15)
    parser.add_argument("--since-window", default="5m")
    args = parser.parse_args()

    print("timestamp, robot_x, robot_y, target_x, target_y, dist_m")
    for _ in range(max(1, args.loops)):
        target = read_latest_target(args.route_compose, args.since_window)
        robot = read_robot_xy(args.sim_compose)
        ts = datetime.now(timezone.utc).isoformat()
        if target is None or robot is None:
            print(f"{ts}, -, -, -, -, -")
        else:
            dist = math.hypot(robot[0] - target[0], robot[1] - target[1])
            print(
                f"{ts}, {robot[0]:.3f}, {robot[1]:.3f}, "
                f"{target[0]:.3f}, {target[1]:.3f}, {dist:.3f}"
            )
        time.sleep(max(0.2, args.interval_sec))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

