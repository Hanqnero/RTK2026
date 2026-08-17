#!/usr/bin/env python3
"""Офлайн-экспорт синхронных raw и bird's-eye кадров из rosbag2/MCAP."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Image


def _parse_arguments() -> argparse.Namespace:
    """Разобрать параметры офлайн-экспорта без запуска ROS graph."""
    parser = argparse.ArgumentParser(
        description=(
            "Сохранить исходные и перспективно преобразованные кадры в PNG, "
            "удалить дубликаты и сопоставить изображения по header.stamp."
        )
    )
    parser.add_argument(
        "--bag",
        required=True,
        type=Path,
        help="Каталог rosbag2 с metadata.yaml и MCAP.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Выходной каталог; по умолчанию <bag>/frame_pairs.",
    )
    parser.add_argument(
        "--raw-topic",
        default="/camera/color/image_raw",
        help="Топик исходного RGB-изображения.",
    )
    parser.add_argument(
        "--projected-topic",
        default="/cv/perspective/image",
        help="Топик bird's-eye изображения.",
    )
    parser.add_argument(
        "--storage-id",
        default="mcap",
        help="rosbag2 storage plugin, по умолчанию mcap.",
    )
    parser.add_argument(
        "--png-compression",
        type=int,
        default=3,
        choices=range(0, 10),
        metavar="[0-9]",
        help="Сжатие PNG: 0 быстрее, 9 меньше файл.",
    )
    return parser.parse_args()


def _message_stamp_ns(message: Image, bag_timestamp_ns: int) -> int:
    """Получить исходное время кадра или использовать время записи."""
    stamp_ns = (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )
    return stamp_ns if stamp_ns > 0 else int(bag_timestamp_ns)


def _write_manifest(
    output_directory: Path,
    raw_frames: dict[int, str],
    projected_frames: dict[int, str],
) -> tuple[int, int, int]:
    """Записать точные пары и полный список синхронизации."""
    paired_stamps = sorted(raw_frames.keys() & projected_frames.keys())
    all_stamps = sorted(raw_frames.keys() | projected_frames.keys())

    with (output_directory / "pairs.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp_ns", "raw_path", "bird_eye_path"])
        for stamp_ns in paired_stamps:
            writer.writerow(
                [
                    stamp_ns,
                    raw_frames[stamp_ns],
                    projected_frames[stamp_ns],
                ]
            )

    with (output_directory / "frames.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "stamp_ns",
                "raw_path",
                "bird_eye_path",
                "is_exact_pair",
            ]
        )
        for stamp_ns in all_stamps:
            writer.writerow(
                [
                    stamp_ns,
                    raw_frames.get(stamp_ns, ""),
                    projected_frames.get(stamp_ns, ""),
                    int(
                        stamp_ns in raw_frames
                        and stamp_ns in projected_frames
                    ),
                ]
            )

    return (
        len(paired_stamps),
        len(raw_frames) - len(paired_stamps),
        len(projected_frames) - len(paired_stamps),
    )


def export_frames(arguments: argparse.Namespace) -> None:
    """Прочитать MCAP последовательно и сохранить уникальные PNG."""
    bag_directory = arguments.bag.resolve()
    if not (bag_directory / "metadata.yaml").is_file():
        raise FileNotFoundError(
            f"В каталоге нет metadata.yaml: {bag_directory}"
        )

    output_directory = (
        arguments.output.resolve()
        if arguments.output is not None
        else bag_directory / "frame_pairs"
    )
    raw_directory = output_directory / "raw"
    projected_directory = output_directory / "bird_eye"
    raw_directory.mkdir(parents=True, exist_ok=True)
    projected_directory.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_directory),
            storage_id=arguments.storage_id,
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
    for topic_name in (
        arguments.raw_topic,
        arguments.projected_topic,
    ):
        if topic_types.get(topic_name) != "sensor_msgs/msg/Image":
            raise RuntimeError(
                f"В bag отсутствует Image-топик {topic_name}"
            )

    bridge = CvBridge()
    frames: dict[str, dict[int, str]] = {
        arguments.raw_topic: {},
        arguments.projected_topic: {},
    }
    duplicate_counts = {
        arguments.raw_topic: 0,
        arguments.projected_topic: 0,
    }
    output_by_topic = {
        arguments.raw_topic: ("raw", raw_directory),
        arguments.projected_topic: ("bird_eye", projected_directory),
    }
    png_options = [cv2.IMWRITE_PNG_COMPRESSION, arguments.png_compression]

    while reader.has_next():
        topic_name, serialized_data, bag_timestamp_ns = reader.read_next()
        if topic_name not in frames:
            continue

        message = deserialize_message(serialized_data, Image)
        stamp_ns = _message_stamp_ns(message, bag_timestamp_ns)
        if stamp_ns in frames[topic_name]:
            duplicate_counts[topic_name] += 1
            continue

        image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        prefix, directory = output_by_topic[topic_name]
        filename = f"{prefix}_{stamp_ns}.png"
        destination = directory / filename
        if not cv2.imwrite(str(destination), image, png_options):
            raise RuntimeError(f"Не удалось записать {destination}")

        frames[topic_name][stamp_ns] = str(
            destination.relative_to(output_directory)
        )
        saved_total = sum(len(values) for values in frames.values())
        if saved_total % 250 == 0:
            print(f"Сохранено уникальных кадров: {saved_total}", flush=True)

    raw_frames = frames[arguments.raw_topic]
    projected_frames = frames[arguments.projected_topic]
    paired_count, raw_only_count, projected_only_count = _write_manifest(
        output_directory,
        raw_frames,
        projected_frames,
    )

    print(f"Готово: {output_directory}")
    print(
        f"raw={len(raw_frames)}, "
        f"bird_eye={len(projected_frames)}, "
        f"точных пар={paired_count}"
    )
    print(
        f"без пары: raw={raw_only_count}, "
        f"bird_eye={projected_only_count}"
    )
    print(
        "Удалено дубликатов: "
        f"raw={duplicate_counts[arguments.raw_topic]}, "
        "bird_eye="
        f"{duplicate_counts[arguments.projected_topic]}"
    )


def main() -> None:
    """Точка входа console_scripts."""
    export_frames(_parse_arguments())


if __name__ == "__main__":
    main()
