#!/usr/bin/env python3
# Relay /camera/image_raw -> /camera/image_projected and /camera/image_compensated
# for TurtleBot3 Autorace pipeline. Resizes to 1000x600 and applies IPM (bird's eye)
# so detect_lane receives a ground-plane view.

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult

# TurtleBot3 detect_lane expects at least 600 rows, window 1000x600
TARGET_HEIGHT = 600
TARGET_WIDTH = 1000


def build_ipm_maps(
    out_width: int,
    out_height: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    camera_height: float,
    y_near: float,
    y_far: float,
    x_left: float,
    x_right: float,
    camera_pitch_rad: float = 0.0,
):
    """
    Build remap arrays for IPM (bird's eye view).
    Output row 0 = far (y_far), row out_height-1 = near (y_near); columns = x_left..x_right.

    Supports optional downward camera pitch (camera_pitch_rad > 0 = nose down).
    For each bird's-eye ground point (y_be forward, x_be lateral, 0 ground):
      Camera frame (Gazebo: +x = optical axis, +y = right, +z = up):
        depth  x_c = y_be * cos(a) + H * sin(a)
        right  y_c = x_be
        up     z_c = y_be * sin(a) - H * cos(a)
      Pixel:   u = cx + fx * y_c / x_c
               v = cy - fy * z_c / x_c   (image v increases downward, camera z is up)
    At pitch=0 this reduces to the standard formula: v = cy + fy*H/y_be.
    """
    cos_a = math.cos(camera_pitch_rad)
    sin_a = math.sin(camera_pitch_rad)

    # Vectorised: rows (j) and columns (i) computed with numpy broadcasting
    t = (out_height - 1 - np.arange(out_height, dtype=np.float64)) / max(out_height - 1, 1)
    y_be = y_near + t * (y_far - y_near)           # shape (H,)
    x_c = y_be * cos_a + camera_height * sin_a     # depth along optical axis, shape (H,)
    z_c = y_be * sin_a - camera_height * cos_a     # vertical in camera frame (up+), shape (H,)
    valid = x_c > 1e-6

    x_be = x_left + (np.arange(out_width, dtype=np.float64) / max(out_width - 1, 1)) * (x_right - x_left)  # shape (W,)

    # map_x[j,i] = cx + fx * x_be[i] / x_c[j]  => broadcast (H,1) and (1,W)
    # map_y[j,i] = cy - fy * z_c[j] / x_c[j]   => shape (H,) broadcast to (H,W)
    x_c_safe = np.where(valid, x_c, 1.0)
    map_x = (cx + fx * x_be[np.newaxis, :] / x_c_safe[:, np.newaxis]).astype(np.float32)
    v_col = (cy - fy * z_c / x_c_safe).astype(np.float32)
    map_y = np.broadcast_to(v_col[:, np.newaxis], (out_height, out_width)).copy().astype(np.float32)

    # Zero out invalid rows so cv2.remap fills them with BORDER_CONSTANT black
    map_x[~valid] = -1.0
    map_y[~valid] = -1.0
    return map_x, map_y


class ImageRelayAutorace(Node):
    def __init__(self):
        super().__init__("image_relay_autorace")
        self.bridge = CvBridge()
        def _fp(lo, hi, step=0.001):
            return ParameterDescriptor(floating_point_range=[
                FloatingPointRange(from_value=lo, to_value=hi, step=step)])

        self.declare_parameter("camera_height_m", 0.025, _fp(0.01, 1.0))
        self.declare_parameter("camera_pitch_rad", 0.0, _fp(-1.5708, 1.5708, step=0.0001))
        self.declare_parameter("y_near_m", 0.08, _fp(0.01, 1.0))
        self.declare_parameter("y_far_m", 1.2, _fp(0.1, 5.0))
        self.declare_parameter("x_left_m", -0.5, _fp(-3.0, 0.0))
        self.declare_parameter("x_right_m", 0.5, _fp(0.0, 3.0))
        self.declare_parameter("use_ipm", True)
        self.declare_parameter("ipm_vertical_flip", False)
        self.add_on_set_parameters_callback(self._on_params_changed)
        # Fallback intrinsics (used only if /camera/camera_info не доступен).
        self.declare_parameter("fx", 593.75)
        self.declare_parameter("fy", 550.0)
        self.declare_parameter("cx", 500.0)
        self.declare_parameter("cy", 300.0)
        self.declare_parameter("projected_topic", "/camera/image_projected")
        self.declare_parameter("compensated_topic", "/camera/image_compensated")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("image_topic", "/camera/image_raw")

        image_topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.cb_image,
            10,
        )

        self._camera_info: CameraInfo | None = None
        self._ipm_maps = None
        self._ipm_maps_key: tuple | None = None

        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.cb_camera_info,
            10,
        )
        projected_topic = self.get_parameter("projected_topic").value
        compensated_topic = self.get_parameter("compensated_topic").value

        self.pub_projected = self.create_publisher(Image, projected_topic, 10)
        self.pub_compensated = self.create_publisher(Image, compensated_topic, 10)
        self.pub_trapezoid = self.create_publisher(Image, "/rtk_autorace_ipm/image_trapezoid", 10)
        self.pub_trapezoid_compressed = self.create_publisher(
            CompressedImage, "/rtk_autorace_ipm/image_trapezoid/compressed", 10
        )
        # Также публикуем JPEG для транспорта image_transport-style вида: <topic>/compressed.
        # Это делает вход для detect_lane более совместимым с реализациями,
        # которые подписываются на .../compressed (например, ROS1 TurtleBot3 Autorace).
        self.pub_projected_compressed = self.create_publisher(
            CompressedImage, projected_topic + "/compressed", 10
        )
        self.pub_compensated_compressed = self.create_publisher(
            CompressedImage, compensated_topic + "/compressed", 10
        )

    def _on_params_changed(self, params):
        ipm_keys = {"camera_height_m", "camera_pitch_rad", "y_near_m", "y_far_m", "x_left_m", "x_right_m", "ipm_vertical_flip"}
        if any(p.name in ipm_keys for p in params):
            self._ipm_maps = None
            self._ipm_maps_key = None
        return SetParametersResult(successful=True)

    def cb_camera_info(self, msg: CameraInfo):
        # Храним только матрицу K (fx/fy/cx/cy) и используем в cb_image для пересчёта под ресайз.
        self._camera_info = msg
        # Принудительно сбрасываем кэш маппинга — вдруг поменялась камера/разрешение.
        self._ipm_maps = None
        self._ipm_maps_key = None

    def _get_ipm_maps(self, raw_width: int, raw_height: int):
        use_ipm = self.get_parameter("use_ipm").value
        if not use_ipm:
            return None

        h_m = self.get_parameter("camera_height_m").value
        pitch = self.get_parameter("camera_pitch_rad").value
        y_near = self.get_parameter("y_near_m").value
        y_far = self.get_parameter("y_far_m").value
        x_left = self.get_parameter("x_left_m").value
        x_right = self.get_parameter("x_right_m").value

        # Intrinsics:
        # - если есть camera_info — берём их в оригинальном разрешении и масштабируем под TARGET_*
        # - иначе — используем параметры fallback.
        if self._camera_info is not None:
            k = self._camera_info.k  # row-major 3x3
            fx0 = float(k[0])
            fy0 = float(k[4])
            cx0 = float(k[2])
            cy0 = float(k[5])
            sx = TARGET_WIDTH / max(raw_width, 1)
            sy = TARGET_HEIGHT / max(raw_height, 1)
            fx = fx0 * sx
            fy = fy0 * sy
            cx = cx0 * sx
            cy = cy0 * sy
        else:
            fx = float(self.get_parameter("fx").value)
            fy = float(self.get_parameter("fy").value)
            cx = float(self.get_parameter("cx").value)
            cy = float(self.get_parameter("cy").value)

        maps_key = (raw_width, raw_height, fx, fy, cx, cy, h_m, pitch, y_near, y_far, x_left, x_right)
        if self._ipm_maps is not None and self._ipm_maps_key == maps_key:
            return self._ipm_maps

        map_x, map_y = build_ipm_maps(
            TARGET_WIDTH,
            TARGET_HEIGHT,
            fx,
            fy,
            cx,
            cy,
            h_m,
            y_near,
            y_far,
            x_left,
            x_right,
            pitch,
        )
        self._ipm_maps = (map_x, map_y)
        self._ipm_maps_key = maps_key
        return self._ipm_maps

    def _ipm_trapezoid_pts(self, raw_width: int, raw_height: int):
        """Return the 4 IPM corners in resized (TARGET_WIDTH x TARGET_HEIGHT) image coords."""
        h_m = self.get_parameter("camera_height_m").value
        pitch = self.get_parameter("camera_pitch_rad").value
        y_near = self.get_parameter("y_near_m").value
        y_far = self.get_parameter("y_far_m").value
        x_left = self.get_parameter("x_left_m").value
        x_right = self.get_parameter("x_right_m").value
        if self._camera_info is not None:
            k = self._camera_info.k
            sx = TARGET_WIDTH / max(raw_width, 1)
            sy = TARGET_HEIGHT / max(raw_height, 1)
            fx = float(k[0]) * sx
            fy = float(k[4]) * sy
            cx = float(k[2]) * sx
            cy = float(k[5]) * sy
        else:
            fx = float(self.get_parameter("fx").value)
            fy = float(self.get_parameter("fy").value)
            cx = float(self.get_parameter("cx").value)
            cy = float(self.get_parameter("cy").value)
        cos_a = math.cos(pitch)
        sin_a = math.sin(pitch)
        pts = []
        for y_be, x_be in [
            (y_near, x_left), (y_near, x_right),
            (y_far, x_right), (y_far, x_left),
        ]:
            x_c = y_be * cos_a + h_m * sin_a
            z_c = y_be * sin_a - h_m * cos_a
            if x_c <= 1e-6:
                pts.append((0, 0))
                continue
            u = cx + fx * x_be / x_c
            v = cy - fy * z_c / x_c
            pts.append((int(round(u)), int(round(v))))
        return pts

    def cb_image(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn("cv_bridge imgmsg_to_cv2: %s" % e)
            return
        h, w = cv_image.shape[:2]
        if h != TARGET_HEIGHT or w != TARGET_WIDTH:
            cv_image = cv2.resize(
                cv_image, (TARGET_WIDTH, TARGET_HEIGHT), interpolation=cv2.INTER_LINEAR
            )
        # Publish trapezoid overlay on original (pre-warp) image
        trap_img = cv_image.copy()
        trap_pts = self._ipm_trapezoid_pts(raw_width=w, raw_height=h)
        cv2.polylines(trap_img, [np.array(trap_pts, dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=3)
        for pt in trap_pts:
            cv2.circle(trap_img, pt, 6, (0, 0, 255), -1)
        trap_msg = self.bridge.cv2_to_imgmsg(trap_img, encoding="bgr8")
        trap_msg.header = msg.header
        self.pub_trapezoid.publish(trap_msg)
        trap_ok, trap_buf = cv2.imencode(".jpg", trap_img)
        if trap_ok:
            trap_comp = CompressedImage()
            trap_comp.header = msg.header
            trap_comp.format = "jpeg"
            trap_comp.data = trap_buf.tobytes()
            self.pub_trapezoid_compressed.publish(trap_comp)
        maps = self._get_ipm_maps(raw_width=w, raw_height=h)
        if maps is not None:
            map_x, map_y = maps
            cv_image = cv2.remap(
                cv_image,
                map_x,
                map_y,
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
            if self.get_parameter("ipm_vertical_flip").value:
                cv_image = cv2.flip(cv_image, 0)
        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        out_msg.header = msg.header
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_projected.publish(out_msg)
        self.pub_compensated.publish(out_msg)

        # Публикуем сжатую версию (JPEG) в отдельные топики.
        # detect_lane может читать either raw, либо compressed, в зависимости от внутренней настройки.
        encode_ok, buf = cv2.imencode(".jpg", cv_image)
        if encode_ok:
            now_stamp = out_msg.header.stamp
            comp_msg = CompressedImage()
            comp_msg.header = msg.header
            comp_msg.header.stamp = now_stamp
            comp_msg.format = "jpeg"
            comp_msg.data = buf.tobytes()
            self.pub_projected_compressed.publish(comp_msg)
            self.pub_compensated_compressed.publish(comp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRelayAutorace()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
