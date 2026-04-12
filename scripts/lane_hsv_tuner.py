#!/usr/bin/env python3
"""
Interactive lane HSV + IPM calibration tool.

Runs INSIDE the container. Shows live HSV mask previews with OpenCV trackbars.
Pushes params to detect_lane and image_relay_autorace nodes on the fly.

Copy into container and run (SEPARATE terminal from autorace launch):
  docker cp scripts/lane_hsv_tuner.py rtk2026_diff_robot_gazebo:/tmp/lane_hsv_tuner.py
  docker exec -it rtk2026_diff_robot_gazebo bash -c \
    "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
     python3 /tmp/lane_hsv_tuner.py"

Keys (click the image window first):
  s  - save lane.yaml to /tmp/lane_tuned.yaml (then docker cp it out)
  a  - apply current HSV to detect_lane node (also happens automatically on slider change)
  i  - apply current IPM params to image_relay_autorace node
  r  - reset HSV to defaults from lane.yaml
  q  - quit

Modes (set via --mode arg):
  sim   (default) - yellow + white lanes (simulation)
  real  - two white lanes (real robot: yellow = same as white)
"""

import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

# ---------------------------------------------------------------------------
# Defaults (loaded from lane.yaml / command line)
# ---------------------------------------------------------------------------
# detect_lane uses cv2.COLOR_BGR2HSV, inRange order: [H, S, V]
# Parameter naming in lane.yaml: "lightness" = V (Value/brightness), NOT HLS lightness.
# H: 0..179  S: 0..255  V: 0..255  (OpenCV HSV)
# White: low saturation (achromatic), high V.  Yellow: H~10-67, high S.
SIM_DEFAULTS = dict(
    w_hl=0, w_hh=179, w_vl=105, w_vh=255, w_sl=0,  w_sh=60,
    y_hl=10, y_hh=67,  y_vl=105, y_vh=255, y_sl=92, y_sh=255,
)
REAL_DEFAULTS = dict(
    w_hl=0, w_hh=179, w_vl=105, w_vh=255, w_sl=0,  w_sh=60,
    y_hl=0, y_hh=179, y_vl=105, y_vh=255, y_sl=0,  y_sh=60,
)
IPM_DEFAULTS = dict(
    h_m=48,       # camera_height_m * 1000 (mm)
    pitch=349,    # camera_pitch_rad * 1000
    y_near=50,    # y_near_m * 1000 (mm)
    y_far=400,    # y_far_m * 1000 (mm)
    x_left=220,   # abs(x_left_m) * 1000 (mm)  [left is negative, stored positive]
    x_right=220,  # x_right_m * 1000 (mm)
)

WIN_MAIN = "Lane HSV Tuner  [s=save  a=apply  i=apply-IPM  r=reset  q=quit]"
WIN_IPM  = "IPM Tuner  [i=apply  q=close]"

DISPLAY_W = 1800   # total width of the composite strip
DISPLAY_H = 360    # height of each image in the strip

# ---------------------------------------------------------------------------

def make_param_int(name: str, value: int) -> Parameter:
    return Parameter(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(value)),
    )

def make_param_float(name: str, value: float) -> Parameter:
    return Parameter(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)),
    )

def make_param_bool(name: str, value: bool) -> Parameter:
    return Parameter(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(value)),
    )


class LaneHSVTuner(Node):
    def __init__(self, mode: str = "sim"):
        super().__init__("lane_hsv_tuner")
        self.bridge = CvBridge()
        self.mode = mode

        defaults = SIM_DEFAULTS if mode == "sim" else REAL_DEFAULTS
        self.hsv = dict(defaults)
        self.ipm = dict(IPM_DEFAULTS)

        self._raw_img: np.ndarray | None = None
        self._ipm_img: np.ndarray | None = None
        self._lock = threading.Lock()

        # Debounce: apply params at most once per 300 ms after slider move
        self._apply_pending = False
        self._apply_ipm_pending = False
        self._last_apply_t = 0.0
        self._last_apply_ipm_t = 0.0

        # ROS clients
        self._lane_client: rclpy.client.Client | None = None
        self._ipm_client: rclpy.client.Client | None = None
        self._clients_ready = False

        self.create_subscription(
            Image, "/camera/image_raw", self._cb_raw, 1
        )
        self.create_subscription(
            Image, "/rtk_autorace_ipm/image_projected", self._cb_ipm, 1
        )

        self._init_clients()

    # ------------------------------------------------------------------
    def _init_clients(self):
        self._lane_client = self.create_client(SetParameters, "/detect_lane/set_parameters")
        self._ipm_client  = self.create_client(SetParameters, "/image_relay_autorace/set_parameters")
        self.create_timer(1.0, self._check_clients)

    def _check_clients(self):
        lane_ok = self._lane_client.service_is_ready()
        ipm_ok  = self._ipm_client.service_is_ready()
        if lane_ok and ipm_ok and not self._clients_ready:
            self.get_logger().info("Both services ready — auto-apply is active.")
            self._clients_ready = True
        elif not lane_ok:
            self.get_logger().warn("Waiting for /detect_lane/set_parameters …", throttle_duration_sec=5.0)
        elif not ipm_ok:
            self.get_logger().warn("Waiting for /image_relay_autorace/set_parameters …", throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    def _cb_raw(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        with self._lock:
            self._raw_img = img

    def _cb_ipm(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        with self._lock:
            self._ipm_img = img

    # ------------------------------------------------------------------
    def _apply_mask_to_image(self, img: np.ndarray) -> np.ndarray:
        """Apply current HSV thresholds and return side-by-side: img | overlay | white | yellow | combined.
        Uses cv2.COLOR_BGR2HSV with inRange order [H, S, V] — same as detect_lane.
        """
        h = self.hsv
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # detect_lane inRange lower/upper: [Hue, Saturation, Lightness(=Value)]
        w_mask = cv2.inRange(
            hsv,
            np.array([h["w_hl"], h["w_sl"], h["w_vl"]], dtype=np.uint8),
            np.array([h["w_hh"], h["w_sh"], h["w_vh"]], dtype=np.uint8),
        )
        y_mask = cv2.inRange(
            hsv,
            np.array([h["y_hl"], h["y_sl"], h["y_vl"]], dtype=np.uint8),
            np.array([h["y_hh"], h["y_sh"], h["y_vh"]], dtype=np.uint8),
        )
        combined = cv2.bitwise_or(w_mask, y_mask)

        # Colour overlays on original
        overlay = img.copy()
        overlay[w_mask > 0] = [255, 255, 255]
        overlay[y_mask > 0] = [0, 255, 255]

        def grey3(m):
            return cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)

        strip = np.hstack([img, overlay, grey3(w_mask), grey3(y_mask), grey3(combined)])
        return strip

    def _build_display(self) -> np.ndarray | None:
        with self._lock:
            raw = self._raw_img.copy() if self._raw_img is not None else None
            ipm = self._ipm_img.copy() if self._ipm_img is not None else None

        source = ipm if ipm is not None else raw
        if source is None:
            return None

        # Resize source to DISPLAY_H
        scale = DISPLAY_H / source.shape[0]
        src_w  = int(source.shape[1] * scale)
        source = cv2.resize(source, (src_w, DISPLAY_H))

        strip = self._apply_mask_to_image(source)
        # Fit strip to DISPLAY_W
        final_scale = DISPLAY_W / strip.shape[1]
        final_h = int(strip.shape[0] * final_scale)
        strip = cv2.resize(strip, (DISPLAY_W, final_h))

        # Labels
        labels = ["IPM image", "Overlay", "White mask", "Yellow mask", "Combined"]
        section_w = DISPLAY_W // len(labels)
        for i, lbl in enumerate(labels):
            cv2.putText(strip, lbl, (i * section_w + 8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

        return strip

    # ------------------------------------------------------------------
    def apply_hsv_to_detect_lane(self):
        if not self._clients_ready:
            self.get_logger().warn("detect_lane not ready yet")
            return
        h = self.hsv
        # Note: yaml uses "lightness" but detect_lane applies it as V (Value) in HSV
        params = [
            make_param_int("detect.lane.white.hue_l",        h["w_hl"]),
            make_param_int("detect.lane.white.hue_h",        h["w_hh"]),
            make_param_int("detect.lane.white.lightness_l",  h["w_vl"]),
            make_param_int("detect.lane.white.lightness_h",  h["w_vh"]),
            make_param_int("detect.lane.white.saturation_l", h["w_sl"]),
            make_param_int("detect.lane.white.saturation_h", h["w_sh"]),
            make_param_int("detect.lane.yellow.hue_l",       h["y_hl"]),
            make_param_int("detect.lane.yellow.hue_h",       h["y_hh"]),
            make_param_int("detect.lane.yellow.lightness_l", h["y_vl"]),
            make_param_int("detect.lane.yellow.lightness_h", h["y_vh"]),
            make_param_int("detect.lane.yellow.saturation_l",h["y_sl"]),
            make_param_int("detect.lane.yellow.saturation_h",h["y_sh"]),
        ]
        req = SetParameters.Request(parameters=params)
        future = self._lane_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info("HSV params applied to detect_lane.")

    def apply_ipm_to_relay(self):
        if not self._clients_ready:
            self.get_logger().warn("image_relay_autorace not ready yet")
            return
        p = self.ipm
        params = [
            make_param_float("camera_height_m",  p["h_m"]    / 1000.0),
            make_param_float("camera_pitch_rad", p["pitch"]  / 1000.0),
            make_param_float("y_near_m",         p["y_near"] / 1000.0),
            make_param_float("y_far_m",          p["y_far"]  / 1000.0),
            make_param_float("x_left_m",        -p["x_left"] / 1000.0),
            make_param_float("x_right_m",        p["x_right"]/ 1000.0),
        ]
        req = SetParameters.Request(parameters=params)
        future = self._ipm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info(
            f"IPM params applied: h={p['h_m']/1000:.3f} pitch={p['pitch']/1000:.4f} "
            f"y_near={p['y_near']/1000:.3f} y_far={p['y_far']/1000:.3f} "
            f"x_left={-p['x_left']/1000:.3f} x_right={p['x_right']/1000:.3f}"
        )

    def save_lane_yaml(self, path="/tmp/lane_tuned.yaml"):
        h = self.hsv
        mode_comment = "# mode: sim (yellow+white)" if self.mode == "sim" else "# mode: real (two white lines)"
        content = f"""{mode_comment}
/detect_lane:
  ros__parameters:
    detect:
      lane:
        white:
          hue_h: {h["w_hh"]}
          hue_l: {h["w_hl"]}
          lightness_h: {h["w_vh"]}
          lightness_l: {h["w_vl"]}
          saturation_h: {h["w_sh"]}
          saturation_l: {h["w_sl"]}
        yellow:
          hue_h: {h["y_hh"]}
          hue_l: {h["y_hl"]}
          lightness_h: {h["y_vh"]}
          lightness_l: {h["y_vl"]}
          saturation_h: {h["y_sh"]}
          saturation_l: {h["y_sl"]}
"""
        with open(path, "w") as f:
            f.write(content)
        p = self.ipm
        ipm_content = f"""# IPM params (paste into launch file or relay node parameters)
camera_height_m:  {p["h_m"]    / 1000.0:.4f}
camera_pitch_rad: {p["pitch"]  / 1000.0:.4f}
y_near_m:         {p["y_near"] / 1000.0:.3f}
y_far_m:          {p["y_far"]  / 1000.0:.3f}
x_left_m:         {-p["x_left"] / 1000.0:.3f}
x_right_m:        {p["x_right"] / 1000.0:.3f}
"""
        ipm_path = path.replace(".yaml", "_ipm.yaml")
        with open(ipm_path, "w") as f:
            f.write(ipm_content)
        self.get_logger().info(
            f"\nSaved!\n"
            f"  lane.yaml → {path}\n"
            f"  ipm params → {ipm_path}\n\n"
            f"Copy out:\n"
            f"  docker cp rtk2026_diff_robot_gazebo:{path} docker/patches/lane.yaml\n"
            f"  docker cp rtk2026_diff_robot_gazebo:{ipm_path} docker/patches/ipm_params.yaml"
        )


# ---------------------------------------------------------------------------
# Trackbar callbacks and main loop
# ---------------------------------------------------------------------------

def nothing(_):
    pass


def setup_hsv_trackbars(win: str, hsv: dict):
    # Trackbar names: H=hue, V=value/brightness (called "lightness" in lane.yaml), S=saturation
    cv2.createTrackbar("W  H_lo",  win, hsv["w_hl"], 179, nothing)
    cv2.createTrackbar("W  H_hi",  win, hsv["w_hh"], 179, nothing)
    cv2.createTrackbar("W  V_lo",  win, hsv["w_vl"], 255, nothing)
    cv2.createTrackbar("W  V_hi",  win, hsv["w_vh"], 255, nothing)
    cv2.createTrackbar("W  S_lo",  win, hsv["w_sl"], 255, nothing)
    cv2.createTrackbar("W  S_hi",  win, hsv["w_sh"], 255, nothing)
    cv2.createTrackbar("Y  H_lo",  win, hsv["y_hl"], 179, nothing)
    cv2.createTrackbar("Y  H_hi",  win, hsv["y_hh"], 179, nothing)
    cv2.createTrackbar("Y  V_lo",  win, hsv["y_vl"], 255, nothing)
    cv2.createTrackbar("Y  V_hi",  win, hsv["y_vh"], 255, nothing)
    cv2.createTrackbar("Y  S_lo",  win, hsv["y_sl"], 255, nothing)
    cv2.createTrackbar("Y  S_hi",  win, hsv["y_sh"], 255, nothing)


def read_hsv_trackbars(win: str) -> dict:
    return dict(
        w_hl=cv2.getTrackbarPos("W  H_lo", win),
        w_hh=cv2.getTrackbarPos("W  H_hi", win),
        w_vl=cv2.getTrackbarPos("W  V_lo", win),
        w_vh=cv2.getTrackbarPos("W  V_hi", win),
        w_sl=cv2.getTrackbarPos("W  S_lo", win),
        w_sh=cv2.getTrackbarPos("W  S_hi", win),
        y_hl=cv2.getTrackbarPos("Y  H_lo", win),
        y_hh=cv2.getTrackbarPos("Y  H_hi", win),
        y_vl=cv2.getTrackbarPos("Y  V_lo", win),
        y_vh=cv2.getTrackbarPos("Y  V_hi", win),
        y_sl=cv2.getTrackbarPos("Y  S_lo", win),
        y_sh=cv2.getTrackbarPos("Y  S_hi", win),
    )


def setup_ipm_trackbars(win: str, ipm: dict):
    cv2.createTrackbar("H mm",      win, ipm["h_m"],    200, nothing)   # 0..200 mm
    cv2.createTrackbar("Pitch*1000",win, ipm["pitch"],  1571, nothing)  # 0..pi/2 * 1000
    cv2.createTrackbar("Y_near mm", win, ipm["y_near"], 500, nothing)
    cv2.createTrackbar("Y_far  mm", win, ipm["y_far"],  1500, nothing)
    cv2.createTrackbar("X_left mm", win, ipm["x_left"], 600, nothing)   # stored positive
    cv2.createTrackbar("X_right mm",win, ipm["x_right"],600, nothing)


def read_ipm_trackbars(win: str) -> dict:
    return dict(
        h_m     =cv2.getTrackbarPos("H mm",       win),
        pitch   =cv2.getTrackbarPos("Pitch*1000", win),
        y_near  =cv2.getTrackbarPos("Y_near mm",  win),
        y_far   =cv2.getTrackbarPos("Y_far  mm",  win),
        x_left  =cv2.getTrackbarPos("X_left mm",  win),
        x_right =cv2.getTrackbarPos("X_right mm", win),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="sim", choices=["sim", "real"],
                        help="sim=yellow+white, real=two white lines")
    args = parser.parse_args()

    rclpy.init()
    node = LaneHSVTuner(mode=args.mode)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # --- Create windows and trackbars ---
    cv2.namedWindow(WIN_MAIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_MAIN, DISPLAY_W, DISPLAY_H + 400)  # extra space for trackbars
    setup_hsv_trackbars(WIN_MAIN, node.hsv)

    show_ipm_win = False

    print(f"\nMode: {args.mode}")
    print("Keys: s=save  a=apply HSV  i=apply IPM  r=reset  q=quit\n")

    DEBOUNCE = 0.3  # seconds between auto-apply calls

    last_hsv = dict(node.hsv)
    last_ipm = dict(node.ipm)

    while True:
        # Read trackbars
        new_hsv = read_hsv_trackbars(WIN_MAIN)

        # Clamp lo <= hi
        new_hsv["w_hh"] = max(new_hsv["w_hh"], new_hsv["w_hl"])
        new_hsv["w_vh"] = max(new_hsv["w_vh"], new_hsv["w_vl"])
        new_hsv["w_sh"] = max(new_hsv["w_sh"], new_hsv["w_sl"])
        new_hsv["y_hh"] = max(new_hsv["y_hh"], new_hsv["y_hl"])
        new_hsv["y_vh"] = max(new_hsv["y_vh"], new_hsv["y_vl"])
        new_hsv["y_sh"] = max(new_hsv["y_sh"], new_hsv["y_sl"])

        node.hsv = new_hsv

        # Auto-apply HSV if changed
        if new_hsv != last_hsv:
            now = time.time()
            if now - node._last_apply_t > DEBOUNCE:
                node.apply_hsv_to_detect_lane()
                node._last_apply_t = now
            last_hsv = dict(new_hsv)

        # IPM window
        if show_ipm_win:
            new_ipm = read_ipm_trackbars(WIN_IPM)
            new_ipm["y_far"] = max(new_ipm["y_far"], new_ipm["y_near"] + 10)
            node.ipm = new_ipm
            if new_ipm != last_ipm:
                now = time.time()
                if now - node._last_apply_ipm_t > DEBOUNCE:
                    node.apply_ipm_to_relay()
                    node._last_apply_ipm_t = now
                last_ipm = dict(new_ipm)

        # Build and show display
        frame = node._build_display()
        if frame is not None:
            cv2.imshow(WIN_MAIN, frame)
        else:
            placeholder = np.zeros((200, DISPLAY_W, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Waiting for /rtk_autorace_ipm/image_projected ...",
                        (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 200), 2)
            cv2.imshow(WIN_MAIN, placeholder)

        key = cv2.waitKey(33) & 0xFF  # ~30 fps

        if key == ord("q"):
            break
        elif key == ord("s"):
            node.save_lane_yaml()
        elif key == ord("a"):
            node.apply_hsv_to_detect_lane()
            node._last_apply_t = time.time()
        elif key == ord("i"):
            if not show_ipm_win:
                # Open IPM window
                cv2.namedWindow(WIN_IPM, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(WIN_IPM, 700, 300)
                setup_ipm_trackbars(WIN_IPM, node.ipm)
                show_ipm_win = True
                print("IPM tuner window opened.")
            else:
                node.apply_ipm_to_relay()
                node._last_apply_ipm_t = time.time()
        elif key == ord("r"):
            defaults = SIM_DEFAULTS if node.mode == "sim" else REAL_DEFAULTS
            node.hsv = dict(defaults)
            # Reset trackbars
            cv2.setTrackbarPos("W  H_lo", WIN_MAIN, defaults["w_hl"])
            cv2.setTrackbarPos("W  H_hi", WIN_MAIN, defaults["w_hh"])
            cv2.setTrackbarPos("W  V_lo", WIN_MAIN, defaults["w_vl"])
            cv2.setTrackbarPos("W  V_hi", WIN_MAIN, defaults["w_vh"])
            cv2.setTrackbarPos("W  S_lo", WIN_MAIN, defaults["w_sl"])
            cv2.setTrackbarPos("W  S_hi", WIN_MAIN, defaults["w_sh"])
            cv2.setTrackbarPos("Y  H_lo", WIN_MAIN, defaults["y_hl"])
            cv2.setTrackbarPos("Y  H_hi", WIN_MAIN, defaults["y_hh"])
            cv2.setTrackbarPos("Y  V_lo", WIN_MAIN, defaults["y_vl"])
            cv2.setTrackbarPos("Y  V_hi", WIN_MAIN, defaults["y_vh"])
            cv2.setTrackbarPos("Y  S_lo", WIN_MAIN, defaults["y_sl"])
            cv2.setTrackbarPos("Y  S_hi", WIN_MAIN, defaults["y_sh"])
            print("HSV reset to defaults.")

    cv2.destroyAllWindows()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print("Tuner closed.")


if __name__ == "__main__":
    main()
