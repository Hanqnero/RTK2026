#!/usr/bin/env python3
"""Direct ONNX traffic-sign detector for ROS camera topics."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtk2026_interfaces.msg import DrivingDetection
from sensor_msgs.msg import CompressedImage, Image

try:
    import onnxruntime as ort
except ImportError:
    ort = None


@dataclass(frozen=True)
class Candidate:
    class_index: int
    class_label: str
    command: str
    confidence: float
    box_area: float
    box: tuple[int, int, int, int]


class OnnxSignDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("onnx_sign_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/perception/driving_detection")
        self.declare_parameter("debug_overlay_topic", "/perception/detection_overlay")
        self.declare_parameter("debug_image_topic", "/perception/detection_overlay/compressed")
        self.declare_parameter("model_path", "/workspace/src/rtk2026_cv/best.onnx")
        self.declare_parameter("input_size", 640)
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("every_n_frames", 2)
        self.declare_parameter("intra_op_num_threads", 2)
        self.declare_parameter("inter_op_num_threads", 1)
        self.declare_parameter("publish_empty", True)
        self.declare_parameter("log_every_n_frames", 30)
        self.declare_parameter("debug_overlay", True)
        self.declare_parameter("stop_duration_sec", 2.0)
        # ~ Каскад лево/право.
        #
        # Основная модель почти не различает turn_left и turn_right - на
        # реальном кадре разрыв уверенности был 46 % против 43 %, монетка.
        # Причина в обучении: fliplr: 0.5 половину кадров отражал, оставляя
        # прежний класс, и сеть выучила игнорировать направление стрелки.
        #
        # Второй, отдельно обученный ONNX смотрит только на вырезанную
        # область знака и обучен без fliplr - см. yolo/finetune_head.py.
        # Пустой путь выключает проверку: старые конфиги без cascade_model_path
        # продолжают работать как раньше, только с прежней слабостью.
        self.declare_parameter("cascade_model_path", "")
        # Насколько увереннее должен быть каскад, чтобы его вердикту вообще
        # доверять. Если разрыв между left и right внутри каскада меньше
        # этого порога, каскад тоже не уверен, и решение основной модели
        # трогать не за что.
        self.declare_parameter("cascade_min_margin", 0.10)
        self.declare_parameter(
            "class_id_to_label",
            [
                "0=bus_stop",
                "1=move_forward",
                "2=no_turn_left",
                "3=no_turn_right",
                "4=obstacle",
                "5=parking_spot",
                "6=turn_left",
                "7=turn_right",
            ],
        )
        self.declare_parameter(
            "class_id_to_route_command",
            [
                "1=straight_only",
                "2=no_left_turn",
                "3=no_right_turn",
                "6=left_only",
                "7=right_only",
            ],
        )
        self.declare_parameter("class_id_to_stop_action", [""])
        self.declare_parameter("bus_class_ids", [""])

        self._frame_count = 0
        self._log_every_n = max(1, int(self.get_parameter("log_every_n_frames").value))
        self._every_n = max(1, int(self.get_parameter("every_n_frames").value))
        self._input_size = int(self.get_parameter("input_size").value)
        self._conf_threshold = float(self.get_parameter("conf_threshold").value)
        self._nms_threshold = float(self.get_parameter("nms_threshold").value)
        self._publish_empty = bool(self.get_parameter("publish_empty").value)
        self._debug_overlay = bool(self.get_parameter("debug_overlay").value)
        self._stop_duration_sec = float(self.get_parameter("stop_duration_sec").value)
        self._cascade_min_margin = float(self.get_parameter("cascade_min_margin").value)
        self._last_log_signature: Optional[tuple[str, str, bool]] = None
        self._last_inference_ms = 0.0
        self._last_source_age_ms: Optional[float] = None

        self._class_id_to_label = self._parse_int_mapping("class_id_to_label")
        self._class_id_to_route_command = self._parse_int_mapping("class_id_to_route_command")
        self._class_id_to_stop_action = self._parse_int_mapping("class_id_to_stop_action")
        self._bus_class_ids = set(self._parse_int_list("bus_class_ids"))

        model_path = Path(str(self.get_parameter("model_path").value))
        if not model_path.exists():
            raise FileNotFoundError(f"ONNX model not found: {model_path}")
        self._ort_session = None
        self._ort_input_name = None
        self._net = None
        if ort is not None:
            session_options = ort.SessionOptions()
            session_options.intra_op_num_threads = max(
                1, int(self.get_parameter("intra_op_num_threads").value)
            )
            session_options.inter_op_num_threads = max(
                1, int(self.get_parameter("inter_op_num_threads").value)
            )
            self._ort_session = ort.InferenceSession(
                str(model_path),
                sess_options=session_options,
                providers=["CPUExecutionProvider"],
            )
            self._ort_input_name = self._ort_session.get_inputs()[0].name
            self.get_logger().info("Using ONNX Runtime backend")
        else:
            cv2.setNumThreads(
                max(1, int(self.get_parameter("intra_op_num_threads").value))
            )
            self._net = cv2.dnn.readNetFromONNX(str(model_path))
            self.get_logger().info("Using OpenCV DNN backend")

        # Каскад грузится отдельной сессией: он маленький (nc=8, но реально
        # обучены только два класса) и падать вместе с основной моделью не
        # должен - при любой проблеме с файлом просто отключается.
        self._cascade_session = None
        self._cascade_input_name = None
        cascade_path_text = str(self.get_parameter("cascade_model_path").value).strip()
        if cascade_path_text and ort is not None:
            cascade_path = Path(cascade_path_text)
            if cascade_path.exists():
                self._cascade_session = ort.InferenceSession(
                    str(cascade_path),
                    providers=["CPUExecutionProvider"],
                )
                self._cascade_input_name = self._cascade_session.get_inputs()[0].name
                self.get_logger().info(f"Каскад лево/право: {cascade_path}")
            else:
                self.get_logger().warning(
                    f"cascade_model_path указан, но файл не найден: {cascade_path} - "
                    "проверка лево/право отключена"
                )

        image_topic = str(self.get_parameter("image_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        debug_overlay_topic = str(self.get_parameter("debug_overlay_topic").value)
        debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self._detection_pub = self.create_publisher(DrivingDetection, output_topic, 10)
        self._debug_overlay_pub = self.create_publisher(Image, debug_overlay_topic, 1)
        self._debug_pub = self.create_publisher(CompressedImage, debug_image_topic, 1)
        # A detector must work on the newest frame. A deeper reliable queue
        # turns overload into seconds of stale inference and is incompatible
        # with camera drivers that publish using sensor-data QoS.
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._image_sub = self.create_subscription(
            Image, image_topic, self._on_image, camera_qos
        )

        self.get_logger().info(
            f"onnx_sign_detector started: image_topic={image_topic}, output_topic={output_topic}, model={model_path}"
        )

    def _parse_int_mapping(self, parameter_name: str) -> dict[int, str]:
        mapping: dict[int, str] = {}
        raw_values = self.get_parameter(parameter_name).value
        for item in raw_values:
            text = str(item).strip()
            if "=" not in text:
                continue
            key_text, value = text.split("=", 1)
            try:
                key = int(key_text.strip())
            except ValueError:
                continue
            value = value.strip()
            if value:
                mapping[key] = value
        return mapping

    def _parse_int_list(self, parameter_name: str) -> list[int]:
        out: list[int] = []
        for item in self.get_parameter(parameter_name).value:
            try:
                out.append(int(str(item).strip()))
            except ValueError:
                continue
        return out

    def _on_image(self, msg: Image) -> None:
        self._frame_count += 1
        if self._frame_count % self._every_n != 0:
            return

        image_bgr = self._image_msg_to_bgr(msg)
        started = perf_counter()
        output = self._forward(image_bgr)
        self._last_inference_ms = (perf_counter() - started) * 1000.0
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        self._last_source_age_ms = (
            max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000.0)
            if stamp_ns > 0
            else None
        )
        best_route: Optional[Candidate] = None
        best_stop: Optional[Candidate] = None
        best_bus: Optional[Candidate] = None

        for candidate in self._iter_candidates(output, image_bgr.shape[1], image_bgr.shape[0]):
            if candidate.command == "bus":
                best_bus = self._pick_better(best_bus, candidate)
            elif candidate.command in self._class_id_to_stop_action.values():
                best_stop = self._pick_better(best_stop, candidate)
            else:
                best_route = self._pick_better(best_route, candidate)

        # Каскад решает только уже принятый спор основной модели -
        # left_only и right_only это единственные команды, за которые
        # отвечают классы 6 и 7 в текущей конфигурации. Найти их по
        # маршрутной команде, а не по числу класса: так проверка не
        # привязана к тому, что turn_left и turn_right - это именно 6 и 7.
        if best_route is not None and best_route.command in ("left_only", "right_only"):
            best_route = self._apply_cascade(best_route, image_bgr)

        if not self._publish_empty and best_route is None and best_stop is None and best_bus is None:
            return

        detection = DrivingDetection()
        detection.header = msg.header
        if best_route is not None:
            detection.route_command = best_route.command
            detection.route_class_id = best_route.class_label
            detection.route_confidence = float(best_route.confidence)
            detection.route_box_area = float(best_route.box_area)
        if best_stop is not None:
            detection.stop_action = best_stop.command
            detection.stop_class_id = best_stop.class_label
            detection.stop_confidence = float(best_stop.confidence)
            detection.stop_box_area = float(best_stop.box_area)
            detection.stop_duration_sec = float(self._stop_duration_sec)
        if best_bus is not None:
            detection.bus_detected = True
            detection.bus_class_id = best_bus.class_label
            detection.bus_confidence = float(best_bus.confidence)
            detection.bus_box_area = float(best_bus.box_area)
        self._detection_pub.publish(detection)

        current_signature = (
            detection.route_command,
            detection.stop_action,
            bool(detection.bus_detected),
        )
        if (best_route is not None or best_stop is not None or best_bus is not None) and current_signature != self._last_log_signature:
            self.get_logger().info(
                "driving_detection "
                f"route={detection.route_command or 'none'}({detection.route_class_id or '-'},{detection.route_confidence:.2f}) "
                f"stop={detection.stop_action or 'none'}({detection.stop_class_id or '-'},{detection.stop_confidence:.2f}) "
                f"bus={'yes' if detection.bus_detected else 'no'}"
            )
        self._last_log_signature = current_signature

        if self._debug_overlay:
            overlay = image_bgr.copy()
            for candidate, color in ((best_route, (0, 255, 0)), (best_stop, (0, 255, 255)), (best_bus, (255, 128, 0))):
                if candidate is None:
                    continue
                x1, y1, x2, y2 = candidate.box
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
                label = f"{candidate.class_label}:{candidate.command} {candidate.confidence:.2f}"
                cv2.putText(overlay, label, (x1, max(16, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            overlay_msg = Image()
            overlay_msg.header = msg.header
            overlay_msg.height = overlay.shape[0]
            overlay_msg.width = overlay.shape[1]
            overlay_msg.encoding = "bgr8"
            overlay_msg.is_bigendian = 0
            overlay_msg.step = int(overlay.shape[1] * overlay.shape[2])
            overlay_msg.data = overlay.tobytes()
            self._debug_overlay_pub.publish(overlay_msg)
            ok, encoded = cv2.imencode(".jpg", overlay)
            if ok:
                debug_msg = CompressedImage()
                debug_msg.header = msg.header
                debug_msg.format = "jpeg"
                debug_msg.data = encoded.tobytes()
                self._debug_pub.publish(debug_msg)

        if self._frame_count % self._log_every_n == 0:
            age = (
                "unknown"
                if self._last_source_age_ms is None
                else f"{self._last_source_age_ms:.0f}ms"
            )
            self.get_logger().info(
                "det "
                f"route={detection.route_command or 'none'} "
                f"stop={detection.stop_action or 'none'} "
                f"bus={'yes' if detection.bus_detected else 'no'} "
                f"inference={self._last_inference_ms:.0f}ms source_age={age}"
            )

    def _forward(self, image_bgr: np.ndarray) -> np.ndarray:
        blob = cv2.dnn.blobFromImage(
            image_bgr,
            scalefactor=1.0 / 255.0,
            size=(self._input_size, self._input_size),
            swapRB=True,
            crop=False,
        )
        if self._ort_session is not None:
            return np.asarray(self._ort_session.run(None, {self._ort_input_name: blob})[0])
        self._net.setInput(blob)
        return np.asarray(self._net.forward())

    def _iter_candidates(self, output: np.ndarray, orig_w: int, orig_h: int):
        if output.ndim == 3 and output.shape[0] == 1:
            output = output[0]
        if output.ndim != 2:
            return

        # This model exports detections as rows: [x1, y1, x2, y2, score, class_id].
        if output.shape[1] == 6:
            yield from self._iter_xyxy_score_class_candidates(output, orig_w, orig_h)
            return
        if output.shape[0] == 6 and output.shape[1] > 6:
            yield from self._iter_xyxy_score_class_candidates(output.T, orig_w, orig_h)
            return

        # Fallback for YOLO-style outputs: [x_c, y_c, w, h, cls...]
        if output.shape[0] < output.shape[1]:
            output = output.T
        if output.shape[1] < 5:
            return

        scale_x = float(orig_w) / float(self._input_size)
        scale_y = float(orig_h) / float(self._input_size)
        raw_candidates: list[tuple[int, float, int, int, int, int]] = []
        for row in output:
            x_c, y_c, w, h = map(float, row[:4])
            class_scores = row[4:]
            class_index = int(np.argmax(class_scores))
            confidence = float(class_scores[class_index])
            if confidence < self._conf_threshold:
                continue

            x1 = int(max(0.0, (x_c - w / 2.0) * scale_x))
            y1 = int(max(0.0, (y_c - h / 2.0) * scale_y))
            x2 = int(min(float(orig_w - 1), (x_c + w / 2.0) * scale_x))
            y2 = int(min(float(orig_h - 1), (y_c + h / 2.0) * scale_y))
            raw_candidates.append((class_index, confidence, x1, y1, x2, y2))

        if not raw_candidates:
            return

        # Suppress duplicates within a class. Running one global NMS would
        # incorrectly discard two different signs whose boxes overlap.
        for class_index in {candidate[0] for candidate in raw_candidates}:
            class_candidates = [
                candidate
                for candidate in raw_candidates
                if candidate[0] == class_index
            ]
            boxes = [
                [x1, y1, x2 - x1, y2 - y1]
                for _, _, x1, y1, x2, y2 in class_candidates
            ]
            scores = [confidence for _, confidence, *_ in class_candidates]
            kept = cv2.dnn.NMSBoxes(
                boxes,
                scores,
                self._conf_threshold,
                self._nms_threshold,
            )
            for index in np.asarray(kept).reshape(-1):
                _, confidence, x1, y1, x2, y2 = class_candidates[int(index)]
                yield from self._make_candidate(
                    class_index, confidence, x1, y1, x2, y2
                )

    def _iter_xyxy_score_class_candidates(self, rows: np.ndarray, orig_w: int, orig_h: int):
        scale_x = float(orig_w) / float(self._input_size)
        scale_y = float(orig_h) / float(self._input_size)
        for row in rows:
            x1, y1, x2, y2, confidence, class_index = row[:6]
            confidence = float(confidence)
            if confidence < self._conf_threshold:
                continue
            class_index = int(class_index)
            sx1 = int(max(0.0, float(x1) * scale_x))
            sy1 = int(max(0.0, float(y1) * scale_y))
            sx2 = int(min(float(orig_w - 1), float(x2) * scale_x))
            sy2 = int(min(float(orig_h - 1), float(y2) * scale_y))
            yield from self._make_candidate(class_index, confidence, sx1, sy1, sx2, sy2)

    def _make_candidate(self, class_index: int, confidence: float, x1: int, y1: int, x2: int, y2: int):
        command = self._class_id_to_route_command.get(class_index)
        if command is None:
            command = self._class_id_to_stop_action.get(class_index)
        if command is None and class_index in self._bus_class_ids:
            command = "bus"
        if command is None:
            return
        if x2 <= x1 or y2 <= y1:
            return

        area = float((x2 - x1) * (y2 - y1))
        label = self._class_id_to_label.get(class_index, str(class_index))
        yield Candidate(
            class_index=class_index,
            class_label=label,
            command=command,
            confidence=confidence,
            box_area=area,
            box=(x1, y1, x2, y2),
        )

    def _apply_cascade(self, route: Candidate, image_bgr: np.ndarray) -> Candidate:
        """Перепроверить left_only/right_only вторым ONNX на целом кадре.

        Каскад намеренно получает весь кадр, а не вырез вокруг рамки основной
        модели. Проверено эмпирически: та же модель на плотном вырезе с полем
        20% давала на порядок меньшую уверенность и путала классы - обучающие
        кадры шли из SAM2-разметки видео целиком, сцена вокруг знака входит в
        то, что модель видела, и вырез сдвигает масштаб знака за пределы
        распределения обучения. Кормить её тем же кадром, что и основную
        модель, дороже по вычислениям, но единственное, что действительно
        работает без переобучения под вырез.

        Возвращает route как есть, если каскад выключен, файл не нашёлся при
        старте или разрыв между left и right внутри каскада меньше
        cascade_min_margin: в этом случае каскад тоже не уверен, и подменять
        решение не на что.
        """

        if self._cascade_session is None:
            return route

        left_class_index = next(
            (idx for idx, cmd in self._class_id_to_route_command.items() if cmd == "left_only"),
            None,
        )
        right_class_index = next(
            (idx for idx, cmd in self._class_id_to_route_command.items() if cmd == "right_only"),
            None,
        )
        if left_class_index is None or right_class_index is None:
            return route

        # Тот же прямой resize без letterbox, что и у основной модели: обе
        # обучены на этом соглашении, менять его здесь означало бы кормить
        # каскад искажением, которого он не видел при обучении.
        blob = cv2.dnn.blobFromImage(
            image_bgr,
            scalefactor=1.0 / 255.0,
            size=(self._input_size, self._input_size),
            swapRB=True,
            crop=False,
        )
        raw = self._cascade_session.run(None, {self._cascade_input_name: blob})[0]
        if raw.ndim == 3 and raw.shape[0] == 1:
            raw = raw[0]
        if raw.shape[0] < raw.shape[1]:
            raw = raw.T
        if raw.shape[1] <= max(left_class_index, right_class_index) + 4:
            return route

        # Максимум по всем якорям отдельно для left и right, а не argmax
        # одного якоря: устойчивее к тому, что лучший якорь для одного
        # класса может не совпасть с лучшим якорем для другого.
        score_left = float(raw[:, 4 + left_class_index].max())
        score_right = float(raw[:, 4 + right_class_index].max())

        if abs(score_left - score_right) < self._cascade_min_margin:
            return route

        cascade_says_left = score_left > score_right
        cascade_confidence = score_left if cascade_says_left else score_right
        cascade_command = "left_only" if cascade_says_left else "right_only"

        if cascade_command == route.command:
            return route

        cascade_class_index = left_class_index if cascade_says_left else right_class_index
        self.get_logger().info(
            f"Каскад поменял вердикт: {route.command} -> {cascade_command} "
            f"(left={score_left:.2f} right={score_right:.2f}, "
            f"была уверенность основной модели {route.confidence:.2f})"
        )
        return Candidate(
            class_index=cascade_class_index,
            class_label=self._class_id_to_label.get(cascade_class_index, cascade_command),
            command=cascade_command,
            confidence=cascade_confidence,
            box_area=route.box_area,
            box=route.box,
        )

    @staticmethod
    def _pick_better(current: Optional[Candidate], new: Candidate) -> Candidate:
        if current is None:
            return new
        if new.box_area > current.box_area:
            return new
        if new.box_area == current.box_area and new.confidence > current.confidence:
            return new
        return current

    @staticmethod
    def _image_msg_to_bgr(msg: Image) -> np.ndarray:
        encoding = str(msg.encoding).lower()
        channels_by_encoding = {
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
            "mono8": 1,
        }
        if encoding not in channels_by_encoding:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        channels = channels_by_encoding[encoding]
        packed_step = int(msg.width) * channels
        step = int(msg.step) or packed_step
        if step < packed_step:
            raise ValueError(
                f"Image step {step} is smaller than packed row {packed_step}"
            )
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, step))
        image = rows[:, :packed_step].reshape((msg.height, msg.width, channels))

        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == "bgr8":
            return image.copy()
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OnnxSignDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
