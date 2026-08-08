#!/usr/bin/env python3
"""Независимая запись автоматически нумеруемых TF-поз."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


SCHEMA = "rtk2026.pose_recording.v1"


def _default_config_path() -> Path:
    """Вернуть установленную конфигурацию независимого набора поз."""
    return (
        Path(get_package_share_directory("rtk2026_observability"))
        / "config"
        / "pose_recording.yaml"
    )


def _arguments() -> argparse.Namespace:
    """Разобрать независимые операции init, capture и status."""
    parser = argparse.ArgumentParser(
        description="Инициализировать и пополнять отдельный набор TF-поз."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=_default_config_path(),
        help="YAML с output_file и именами TF frames.",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    initialize = commands.add_parser(
        "init",
        help="Инициализировать пустой набор.",
    )
    initialize.add_argument(
        "--force",
        action="store_true",
        help="Явно очистить существующий набор.",
    )
    commands.add_parser("capture", help="Добавить текущую TF-позу.")
    commands.add_parser("status", help="Показать состояние набора.")
    return parser.parse_args()


def _load_config(path: Path) -> dict[str, Any]:
    """Загрузить настройки хранения и TF без значений в Python-коде."""
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict):
        raise ValueError("Корень конфигурации должен быть YAML mapping")

    required = {
        "output_file",
        "target_frame",
        "source_frame",
        "timeout_sec",
    }
    missing = required.difference(config)
    if missing:
        raise ValueError(
            f"В конфигурации отсутствуют поля: {sorted(missing)}"
        )
    return config


def _output_path(config: dict[str, Any]) -> Path:
    """Развернуть переменные окружения в пути набора поз."""
    return Path(
        os.path.expandvars(
            os.path.expanduser(str(config["output_file"]))
        )
    )


def _new_document(config: dict[str, Any]) -> dict[str, Any]:
    """Создать пустой документ без пользовательских имён точек."""
    return {
        "schema": SCHEMA,
        "target_frame": str(config["target_frame"]),
        "source_frame": str(config["source_frame"]),
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "updated_at_utc": None,
        "samples": [],
    }


def _load_document(
    path: Path,
    config: dict[str, Any],
) -> dict[str, Any]:
    """Загрузить и проверить существующий набор поз."""
    if not path.exists():
        raise RuntimeError(
            "Набор поз не инициализирован; сначала выполни команду init"
        )

    with path.open("r", encoding="utf-8") as stream:
        document = json.load(stream)

    if document.get("schema") != SCHEMA:
        raise RuntimeError("Неподдерживаемая схема JSON")
    if document.get("target_frame") != str(config["target_frame"]):
        raise RuntimeError("target_frame JSON не совпадает с конфигурацией")
    if document.get("source_frame") != str(config["source_frame"]):
        raise RuntimeError("source_frame JSON не совпадает с конфигурацией")
    if not isinstance(document.get("samples"), list):
        raise RuntimeError("Поле samples должно быть JSON array")
    return document


def _write_atomic(path: Path, document: dict[str, Any]) -> None:
    """Атомарно заменить JSON, не оставляя частично записанный файл."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=path.parent,
        text=True,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(
                document,
                stream,
                ensure_ascii=False,
                indent=2,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def _yaw_from_quaternion(rotation: Any) -> float:
    """Вычислить planar yaw из quaternion ROS."""
    return math.atan2(
        2.0
        * (
            rotation.w * rotation.z
            + rotation.x * rotation.y
        ),
        1.0
        - 2.0
        * (
            rotation.y * rotation.y
            + rotation.z * rotation.z
        ),
    )


def _lookup_transform(node: Node, config: dict[str, Any]):
    """Получить последнюю TF, ожидая не дольше timeout_sec."""
    target_frame = str(config["target_frame"])
    source_frame = str(config["source_frame"])
    timeout_sec = float(config["timeout_sec"])

    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    # Ссылка сохраняет listener до завершения lookup.
    node._pose_transform_listener = listener

    deadline = time.monotonic() + max(0.1, timeout_sec)
    last_error = ""
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            return buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
        except TransformException as error:
            last_error = str(error)

    raise RuntimeError(
        f"TF {target_frame} -> {source_frame} не получена "
        f"за {timeout_sec:.1f} с: {last_error or 'нет данных'}"
    )


def _initialize(
    path: Path,
    config: dict[str, Any],
    force: bool,
) -> None:
    """Инициализировать отдельный набор, защищая старые данные."""
    if path.exists() and not force:
        raise RuntimeError(
            f"Файл уже существует: {path}; для очистки укажи init --force"
        )
    _write_atomic(path, _new_document(config))
    print(f"Набор поз инициализирован: {path}")


def _capture(path: Path, config: dict[str, Any]) -> None:
    """Добавить текущую позу с автоматически назначенным sequence."""
    document = _load_document(path, config)

    rclpy.init()
    node = Node("pose_recorder")
    try:
        transform = _lookup_transform(node, config)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = _yaw_from_quaternion(rotation)
    sequence = len(document["samples"])

    document["samples"].append(
        {
            "sequence": sequence,
            "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
            "tf_stamp": {
                "sec": int(transform.header.stamp.sec),
                "nanosec": int(transform.header.stamp.nanosec),
            },
            "position": {
                "x": float(translation.x),
                "y": float(translation.y),
                "z": float(translation.z),
            },
            "orientation": {
                "x": float(rotation.x),
                "y": float(rotation.y),
                "z": float(rotation.z),
                "w": float(rotation.w),
            },
            "pose_2d": {
                "x": float(translation.x),
                "y": float(translation.y),
                "theta": float(yaw),
            },
            "yaw_rad": float(yaw),
            "yaw_deg": float(math.degrees(yaw)),
        }
    )
    document["updated_at_utc"] = datetime.now(timezone.utc).isoformat()
    _write_atomic(path, document)
    print(
        f"Поза {sequence}: x={translation.x:.3f} м, "
        f"y={translation.y:.3f} м, yaw={yaw:.3f} рад"
    )


def _show_status(path: Path, config: dict[str, Any]) -> None:
    """Показать frames, путь и число сохранённых поз."""
    document = _load_document(path, config)
    print(f"Файл: {path}")
    print(
        f"TF: {document['target_frame']} -> "
        f"{document['source_frame']}"
    )
    print(f"Количество поз: {len(document['samples'])}")


def main() -> None:
    """Выполнить выбранную независимую операцию."""
    arguments = _arguments()
    try:
        config = _load_config(arguments.config)
        path = _output_path(config)
        if arguments.command == "init":
            _initialize(path, config, arguments.force)
        elif arguments.command == "capture":
            _capture(path, config)
        elif arguments.command == "status":
            _show_status(path, config)
    except (
        OSError,
        ValueError,
        RuntimeError,
        json.JSONDecodeError,
        yaml.YAMLError,
    ) as error:
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    main()
