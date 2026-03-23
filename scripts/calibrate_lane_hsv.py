#!/usr/bin/env python3
"""
Auto-calibrate HLS lane detection parameters for detect_lane node.

Runs INSIDE the container while detect_lane is running in calibration mode:
  .\scripts\run_rtk2026_diff_robot.ps1 -World autorace -Autorace -NoMap -LaneCalibration

Copy into the container and run:
  docker cp scripts/calibrate_lane_hsv.py rtk2026_diff_robot_gazebo:/tmp/calibrate_lane_hsv.py
  docker exec -it rtk2026_diff_robot_gazebo bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 /tmp/calibrate_lane_hsv.py"

When done, save parameters to lane.yaml:
  docker cp rtk2026_diff_robot_gazebo:/workspace/install/turtlebot3_autorace_detect/share/turtlebot3_autorace_detect/param/lane/lane.yaml docker/patches/lane.yaml

Strategy:
  - Try presets for white (narrow → wide lightness range) combined with yellow presets
  - Monitor /detect/lane_state for 2s after each combination
  - Stop when lane_state == 3 appears >= 70% of readings
  - At the end, apply the best combination and print the lane.yaml snippet
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import UInt8


# ---------------------------------------------------------------------------
# HLS in OpenCV: H=0..179, L=0..255, S=0..255
# Presets: (hue_l, hue_h, lightness_l, lightness_h, saturation_l, saturation_h)
# White: achromatic, high L, low S. Key knob = lightness_l.
# Yellow: H ~15-40 in OpenCV (yellow = 30 in 0-179 scale), mid L, high S.
# ---------------------------------------------------------------------------
WHITE_PRESETS = [
    # strict → permissive (will stop as soon as score is good enough)
    (0, 179, 195, 255, 0,  40),
    (0, 179, 185, 255, 0,  50),
    (0, 179, 175, 255, 0,  55),
    (0, 179, 165, 255, 0,  60),
    (0, 179, 155, 255, 0,  65),
    (0, 179, 145, 255, 0,  70),
    (0, 179, 135, 255, 0,  70),
    (0, 179, 120, 255, 0,  70),
    (0, 179, 120, 255, 0,  60),  # original lane.yaml
]

YELLOW_PRESETS = [
    # narrow hue → wider; saturation_l varies
    (15, 40,  100, 220, 130, 255),
    (12, 45,   95, 230, 120, 255),
    (10, 50,   90, 240, 110, 255),
    (10, 55,   86, 255, 100, 255),
    (10, 60,   86, 255,  92, 255),
    (10, 67,   86, 255,  92, 255),  # original lane.yaml
    ( 5, 67,   80, 255,  80, 255),
    ( 5, 70,   75, 255,  75, 255),
]


class LaneCalibrator(Node):
    def __init__(self):
        super().__init__("lane_calibrator")
        self._lock = threading.Lock()
        self._state_buf: list[int] = []

        self.create_subscription(UInt8, "/detect/lane_state", self._cb_state, 10)

        self._param_client = self.create_client(
            SetParameters, "/detect_lane/set_parameters"
        )
        self.get_logger().info("Waiting for /detect_lane/set_parameters …")
        if not self._param_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(
                "detect_lane service not available. "
                "Is detect_lane running in calibration mode?"
            )
            sys.exit(1)
        self.get_logger().info("Service ready. Starting calibration.")

    # ------------------------------------------------------------------
    def _cb_state(self, msg: UInt8):
        with self._lock:
            self._state_buf.append(int(msg.data))
            if len(self._state_buf) > 200:
                self._state_buf = self._state_buf[-200:]

    def _observe(self, duration_sec: float = 2.0) -> list[int]:
        with self._lock:
            self._state_buf.clear()
        deadline = time.time() + duration_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        with self._lock:
            return list(self._state_buf)

    def _score(self, states: list[int]) -> tuple[float, float]:
        """Returns (frac_state3, frac_state_ge2). Higher is better."""
        if not states:
            return 0.0, 0.0
        n = len(states)
        return (
            sum(1 for s in states if s == 3) / n,
            sum(1 for s in states if s >= 2) / n,
        )

    # ------------------------------------------------------------------
    def _set_int(self, name: str, value: int):
        req = SetParameters.Request()
        pv = ParameterValue(
            type=ParameterType.PARAMETER_INTEGER, integer_value=int(value)
        )
        req.parameters = [Parameter(name=name, value=pv)]
        future = self._param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def _apply(self, color: str, preset: tuple):
        hl, hh, ll, lh, sl, sh = preset
        base = f"detect.lane.{color}"
        self._set_int(f"{base}.hue_l",       hl)
        self._set_int(f"{base}.hue_h",       hh)
        self._set_int(f"{base}.lightness_l", ll)
        self._set_int(f"{base}.lightness_h", lh)
        self._set_int(f"{base}.saturation_l", sl)
        self._set_int(f"{base}.saturation_h", sh)

    # ------------------------------------------------------------------
    def calibrate(self):
        best = {"white": None, "yellow": None, "score": (-1.0, -1.0)}

        total = len(WHITE_PRESETS) * len(YELLOW_PRESETS)
        self.get_logger().info(
            f"Testing {len(WHITE_PRESETS)} white × {len(YELLOW_PRESETS)} yellow "
            f"= {total} combinations (2 s each ≈ {total * 2 // 60} min)"
        )

        for wi, wp in enumerate(WHITE_PRESETS):
            self._apply("white", wp)
            for yi, yp in enumerate(YELLOW_PRESETS):
                self._apply("yellow", yp)
                states = self._observe(2.0)
                s3, s2 = self._score(states)

                self.get_logger().info(
                    f"[W{wi:02d}|Y{yi:02d}] "
                    f"white={wp}  yellow={yp} "
                    f"→ state3={s3:.0%}  state≥2={s2:.0%}  (n={len(states)})"
                )

                if (s3, s2) > best["score"]:
                    best.update(white=wp, yellow=yp, score=(s3, s2))

                if s3 >= 0.70:
                    self.get_logger().info(
                        "lane_state == 3 achieved ≥ 70 %% — stopping search early."
                    )
                    self._finish(best)
                    return

        self.get_logger().info("Search complete (no early stop).")
        self._finish(best)

    # ------------------------------------------------------------------
    def _finish(self, best: dict):
        wp = best["white"]
        yp = best["yellow"]
        s3, s2 = best["score"]

        if wp is None:
            self.get_logger().warn("No improvement found — keeping original params.")
            return

        # Apply best parameters
        self._apply("white", wp)
        self._apply("yellow", yp)

        hl, hh, ll, lh, sl, sh = wp
        yl, yh, yll, ylh, ysl, ysh = yp

        yaml_snippet = f"""
/detect_lane:
  ros__parameters:
    detect:
      lane:
        white:
          hue_h: {hh}
          hue_l: {hl}
          lightness_h: {lh}
          lightness_l: {ll}
          saturation_h: {sh}
          saturation_l: {sl}
        yellow:
          hue_h: {yh}
          hue_l: {yl}
          lightness_h: {ylh}
          lightness_l: {yll}
          saturation_h: {ysh}
          saturation_l: {ysl}
"""
        self.get_logger().info(
            f"\n{'=' * 60}\n"
            f"BEST RESULT  lane_state==3 rate: {s3:.0%}  state>=2: {s2:.0%}\n"
            f"{'=' * 60}\n"
            f"lane.yaml snippet:\n{yaml_snippet}"
            f"\nSave with:\n"
            f"  docker cp rtk2026_diff_robot_gazebo:"
            f"/workspace/install/turtlebot3_autorace_detect/share/"
            f"turtlebot3_autorace_detect/param/lane/lane.yaml "
            f"docker/patches/lane.yaml\n"
        )


def main():
    rclpy.init()
    node = LaneCalibrator()
    try:
        node.calibrate()
    except KeyboardInterrupt:
        node.get_logger().info("Calibration interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
