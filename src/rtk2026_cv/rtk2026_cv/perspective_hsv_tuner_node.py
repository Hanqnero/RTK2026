#!/usr/bin/env python3
"""Интерактивная настройка перспективного преобразования и HSV-фильтра."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml


PREVIEW_WINDOW = "RTK2026 Perspective + HSV"
CONTROLS_WINDOW = "RTK2026 Perspective + HSV controls"

# Координаты четырёх углов задаются в долях ширины и высоты кадра.
# Такой конфиг остаётся применимым при смене разрешения камеры.
POINT_PARAMETERS = (
    "top_left_x",
    "top_left_y",
    "top_right_x",
    "top_right_y",
    "bottom_right_x",
    "bottom_right_y",
    "bottom_left_x",
    "bottom_left_y",
)

HSV_PARAMETERS = (
    "h_min",
    "h_max",
    "s_min",
    "s_max",
    "v_min",
    "v_max",
)

DEFAULT_TUNING = {
    "top_left_x": 0.35,
    "top_left_y": 0.52,
    "top_right_x": 0.65,
    "top_right_y": 0.52,
    "bottom_right_x": 0.95,
    "bottom_right_y": 0.95,
    "bottom_left_x": 0.05,
    "bottom_left_y": 0.95,
    "h_min": 0,
    "h_max": 179,
    "s_min": 0,
    "s_max": 255,
    "v_min": 0,
    "v_max": 255,
}

TRACKBARS = {
    "TL x /1000": ("top_left_x", 1000),
    "TL y /1000": ("top_left_y", 1000),
    "TR x /1000": ("top_right_x", 1000),
    "TR y /1000": ("top_right_y", 1000),
    "BR x /1000": ("bottom_right_x", 1000),
    "BR y /1000": ("bottom_right_y", 1000),
    "BL x /1000": ("bottom_left_x", 1000),
    "BL y /1000": ("bottom_left_y", 1000),
    "H min": ("h_min", 179),
    "H max": ("h_max", 179),
    "S min": ("s_min", 255),
    "S max": ("s_max", 255),
    "V min": ("v_min", 255),
    "V max": ("v_max", 255),
}


def _trackbar_callback(_: int) -> None:
    """Callback OpenCV: значения считываются централизованно в GUI-цикле."""


class PerspectiveHsvTunerNode(Node):
    """Преобразует RGB-кадр в bird's-eye view и настраивает HSV-маску."""

    def __init__(self) -> None:
        super().__init__("perspective_hsv_tuner")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("projected_topic", "/cv/perspective/image")
        self.declare_parameter("mask_topic", "/cv/perspective/hsv_mask")
        self.declare_parameter("filtered_topic", "/cv/perspective/hsv_filtered")
        self.declare_parameter("debug_topic", "/cv/perspective/debug")
        self.declare_parameter("output_width", 640)
        self.declare_parameter("output_height", 480)
        self.declare_parameter("morphology_kernel", 3)
        self.declare_parameter("show_gui", True)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter(
            "config_output_path",
            "/workspace/records/cv/perspective_hsv_tuned.yaml",
        )

        for name in POINT_PARAMETERS:
            self.declare_parameter(name, float(DEFAULT_TUNING[name]))
        for name in HSV_PARAMETERS:
            self.declare_parameter(name, int(DEFAULT_TUNING[name]))

        self._tuning: dict[str, float | int] = {
            name: self.get_parameter(name).value
            for name in (*POINT_PARAMETERS, *HSV_PARAMETERS)
        }
        self._frame_index = 0
        self._last_debug_image: np.ndarray | None = None
        self._last_error = ""
        self._gui_initialized = False
        self._updating_trackbars = False

        self._bridge = CvBridge()
        self.add_on_set_parameters_callback(self._on_parameters)

        image_topic = str(self.get_parameter("image_topic").value)
        self._projected_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("projected_topic").value),
            qos_profile_sensor_data,
        )
        self._mask_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("mask_topic").value),
            qos_profile_sensor_data,
        )
        self._filtered_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("filtered_topic").value),
            qos_profile_sensor_data,
        )
        self._debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("debug_topic").value),
            qos_profile_sensor_data,
        )
        self._image_subscription = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Perspective/HSV tuner запущен: "
            f"input={image_topic}, gui={self.show_gui}"
        )

    @property
    def show_gui(self) -> bool:
        """Нужно ли создавать интерактивные окна OpenCV."""
        return bool(self.get_parameter("show_gui").value)

    def _on_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        """Проверить runtime-параметры и обновить рабочие значения."""
        candidate = dict(self._tuning)

        for parameter in parameters:
            if parameter.name in POINT_PARAMETERS:
                value = float(parameter.value)
                if not 0.0 <= value <= 1.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} должен находиться в [0, 1]",
                    )
                candidate[parameter.name] = value
            elif parameter.name in HSV_PARAMETERS:
                value = int(parameter.value)
                maximum = 179 if parameter.name.startswith("h_") else 255
                if not 0 <= value <= maximum:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} должен находиться в [0, {maximum}]",
                    )
                candidate[parameter.name] = value

        self._tuning = candidate
        return SetParametersResult(successful=True)

    def _source_points(self, width: int, height: int) -> np.ndarray:
        """Преобразовать нормализованный четырёхугольник в пиксели кадра."""
        p = self._tuning
        return np.asarray(
            [
                [p["top_left_x"] * (width - 1), p["top_left_y"] * (height - 1)],
                [p["top_right_x"] * (width - 1), p["top_right_y"] * (height - 1)],
                [
                    p["bottom_right_x"] * (width - 1),
                    p["bottom_right_y"] * (height - 1),
                ],
                [
                    p["bottom_left_x"] * (width - 1),
                    p["bottom_left_y"] * (height - 1),
                ],
            ],
            dtype=np.float32,
        )

    def _hsv_mask(self, projected: np.ndarray) -> np.ndarray:
        """Построить HSV-маску, включая диапазон hue через границу 179→0."""
        p = self._tuning
        hsv = cv2.cvtColor(projected, cv2.COLOR_BGR2HSV)
        h_min = int(p["h_min"])
        h_max = int(p["h_max"])
        lower_sv = (int(p["s_min"]), int(p["v_min"]))
        upper_sv = (int(p["s_max"]), int(p["v_max"]))

        if h_min <= h_max:
            mask = cv2.inRange(
                hsv,
                np.array([h_min, *lower_sv], dtype=np.uint8),
                np.array([h_max, *upper_sv], dtype=np.uint8),
            )
        else:
            # Красный цвет часто пересекает границу шкалы OpenCV Hue.
            low_range = cv2.inRange(
                hsv,
                np.array([0, *lower_sv], dtype=np.uint8),
                np.array([h_max, *upper_sv], dtype=np.uint8),
            )
            high_range = cv2.inRange(
                hsv,
                np.array([h_min, *lower_sv], dtype=np.uint8),
                np.array([179, *upper_sv], dtype=np.uint8),
            )
            mask = cv2.bitwise_or(low_range, high_range)

        kernel_size = max(0, int(self.get_parameter("morphology_kernel").value))
        if kernel_size > 1:
            if kernel_size % 2 == 0:
                kernel_size += 1
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (kernel_size, kernel_size),
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    @staticmethod
    def _label(image: np.ndarray, text: str) -> np.ndarray:
        """Добавить подпись в левый верхний угол отладочного кадра."""
        result = image.copy()
        cv2.rectangle(result, (0, 0), (result.shape[1], 32), (0, 0, 0), -1)
        cv2.putText(
            result,
            text,
            (8, 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        return result

    def _build_debug_image(
        self,
        original: np.ndarray,
        points: np.ndarray,
        projected: np.ndarray,
        mask: np.ndarray,
        filtered: np.ndarray,
    ) -> np.ndarray:
        """Собрать окно original/trapezoid, bird's-eye, mask и результат."""
        overlay = original.copy()
        polygon = np.rint(points).astype(np.int32)
        cv2.polylines(overlay, [polygon], True, (0, 255, 0), 2)
        for index, point in enumerate(polygon):
            cv2.circle(overlay, tuple(point), 5, (0, 0, 255), -1)
            cv2.putText(
                overlay,
                ("TL", "TR", "BR", "BL")[index],
                tuple(point + np.array([6, -6])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

        tile_width = 480
        tile_height = 360

        def tile(image: np.ndarray, label: str) -> np.ndarray:
            resized = cv2.resize(
                image,
                (tile_width, tile_height),
                interpolation=cv2.INTER_AREA,
            )
            return self._label(resized, label)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        return np.vstack(
            [
                np.hstack(
                    [
                        tile(overlay, "Source quadrilateral"),
                        tile(projected, "Bird's-eye projection"),
                    ]
                ),
                np.hstack(
                    [
                        tile(mask_bgr, "HSV mask"),
                        tile(filtered, "HSV filtered"),
                    ]
                ),
            ]
        )

    def _publish_image(
        self,
        publisher: Any,
        image: np.ndarray,
        encoding: str,
        source_message: Image,
    ) -> None:
        """Опубликовать OpenCV-изображение, сохранив timestamp исходного кадра."""
        message = self._bridge.cv2_to_imgmsg(image, encoding=encoding)
        message.header = source_message.header
        publisher.publish(message)

    def _on_image(self, message: Image) -> None:
        """Обработать очередной кадр камеры."""
        self._frame_index += 1
        every_n = max(
            1,
            int(self.get_parameter("process_every_n_frames").value),
        )
        if self._frame_index % every_n != 0:
            return

        try:
            original = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
            height, width = original.shape[:2]
            points = self._source_points(width, height)
            if abs(cv2.contourArea(points)) < 25.0:
                raise ValueError("четырёхугольник проекции имеет слишком малую площадь")

            output_width = max(
                32,
                int(self.get_parameter("output_width").value),
            )
            output_height = max(
                32,
                int(self.get_parameter("output_height").value),
            )
            destination = np.asarray(
                [
                    [0, 0],
                    [output_width - 1, 0],
                    [output_width - 1, output_height - 1],
                    [0, output_height - 1],
                ],
                dtype=np.float32,
            )
            transform = cv2.getPerspectiveTransform(points, destination)
            projected = cv2.warpPerspective(
                original,
                transform,
                (output_width, output_height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
            mask = self._hsv_mask(projected)
            filtered = cv2.bitwise_and(projected, projected, mask=mask)
            debug = self._build_debug_image(
                original,
                points,
                projected,
                mask,
                filtered,
            )

            self._publish_image(
                self._projected_publisher,
                projected,
                "bgr8",
                message,
            )
            self._publish_image(
                self._mask_publisher,
                mask,
                "mono8",
                message,
            )
            self._publish_image(
                self._filtered_publisher,
                filtered,
                "bgr8",
                message,
            )
            if bool(self.get_parameter("publish_debug").value):
                self._publish_image(
                    self._debug_publisher,
                    debug,
                    "bgr8",
                    message,
                )

            self._last_debug_image = debug
            self._last_error = ""
        except (ValueError, cv2.error) as error:
            text = str(error)
            if text != self._last_error:
                self.get_logger().warn(f"Кадр не обработан: {text}")
                self._last_error = text

    def _initialize_gui(self) -> None:
        """Создать окна и заполнить ползунки текущими параметрами."""
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(PREVIEW_WINDOW, 960, 720)
        cv2.namedWindow(CONTROLS_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CONTROLS_WINDOW, 640, 640)

        for label, (parameter_name, maximum) in TRACKBARS.items():
            value = self._tuning[parameter_name]
            position = (
                int(round(float(value) * 1000))
                if parameter_name in POINT_PARAMETERS
                else int(value)
            )
            cv2.createTrackbar(
                label,
                CONTROLS_WINDOW,
                position,
                maximum,
                _trackbar_callback,
            )
        self._gui_initialized = True

    def _read_gui_parameters(self) -> None:
        """Перенести изменившиеся положения ползунков в ROS-параметры."""
        if self._updating_trackbars:
            return

        changed: list[Parameter] = []
        for label, (parameter_name, _) in TRACKBARS.items():
            position = cv2.getTrackbarPos(label, CONTROLS_WINDOW)
            value: float | int = (
                position / 1000.0
                if parameter_name in POINT_PARAMETERS
                else position
            )
            if value != self._tuning[parameter_name]:
                changed.append(Parameter(parameter_name, value=value))

        if changed:
            results = self.set_parameters(changed)
            for parameter, result in zip(changed, results):
                if not result.successful:
                    self.get_logger().warn(
                        f"{parameter.name} не применён: {result.reason}"
                    )

    def _set_trackbars_from_tuning(self) -> None:
        """Обновить GUI после сброса значений."""
        self._updating_trackbars = True
        try:
            for label, (parameter_name, _) in TRACKBARS.items():
                value = self._tuning[parameter_name]
                position = (
                    int(round(float(value) * 1000))
                    if parameter_name in POINT_PARAMETERS
                    else int(value)
                )
                cv2.setTrackbarPos(label, CONTROLS_WINDOW, position)
        finally:
            self._updating_trackbars = False

    def reset_tuning(self) -> None:
        """Вернуть исходный четырёхугольник и полный HSV-диапазон."""
        parameters = [
            Parameter(name, value=value)
            for name, value in DEFAULT_TUNING.items()
        ]
        self.set_parameters(parameters)
        self._set_trackbars_from_tuning()
        self.get_logger().info("Параметры настройки сброшены")

    def save_tuning(self) -> Path:
        """Сохранить текущие значения как ROS 2 parameters YAML."""
        output_path = Path(
            str(self.get_parameter("config_output_path").value)
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)

        parameters: dict[str, Any] = {
            name: self._tuning[name]
            for name in (*POINT_PARAMETERS, *HSV_PARAMETERS)
        }
        for name in (
            "image_topic",
            "projected_topic",
            "mask_topic",
            "filtered_topic",
            "debug_topic",
            "output_width",
            "output_height",
            "morphology_kernel",
            "publish_debug",
            "process_every_n_frames",
        ):
            parameters[name] = self.get_parameter(name).value
        parameters["show_gui"] = False
        parameters["config_output_path"] = str(output_path)

        with output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(
                {
                    "perspective_hsv_tuner": {
                        "ros__parameters": parameters,
                    }
                },
                stream,
                allow_unicode=True,
                sort_keys=False,
            )
        self.get_logger().info(f"Настройка сохранена: {output_path}")
        return output_path

    def run_gui(self) -> None:
        """Выполнять ROS callbacks и обслуживать HighGUI в одном потоке."""
        self._initialize_gui()
        placeholder = np.zeros((720, 960, 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            "Waiting for camera image...",
            (220, 360),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        self.get_logger().info(
            "Управление: s — сохранить YAML, r — сбросить, q — выйти"
        )
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                self._read_gui_parameters()
                image = (
                    self._last_debug_image
                    if self._last_debug_image is not None
                    else placeholder
                )
                cv2.imshow(PREVIEW_WINDOW, image)
                controls = np.zeros((80, 640, 3), dtype=np.uint8)
                cv2.putText(
                    controls,
                    "s: save   r: reset   q: quit",
                    (12, 48),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(CONTROLS_WINDOW, controls)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    self.save_tuning()
                elif key == ord("r"):
                    self.reset_tuning()
        finally:
            cv2.destroyAllWindows()


def main(args: list[str] | None = None) -> None:
    """Запустить интерактивную либо headless-ноду обработки."""
    rclpy.init(args=args)
    node = PerspectiveHsvTunerNode()
    try:
        if node.show_gui:
            node.run_gui()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
