#!/usr/bin/env python3
"""Генерация текстуры стандартного маркера OpenCV ArUco.

Скрипт не хранит и не формирует матрицу маркера вручную. Код выбирается
OpenCV из предопределённого словаря по имени и числовому идентификатору.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "aruco_marker"
    / "materials"
    / "textures"
    / "aruco_4x4_50_id_0.png"
)


def parse_args() -> argparse.Namespace:
    """Разобрать параметры воспроизводимой генерации текстуры."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--id", type=int, default=0, dest="marker_id")
    parser.add_argument("--pixels", type=int, default=1024)
    parser.add_argument(
        "--margin",
        type=int,
        default=128,
        help="Белое поле вокруг маркера в пикселях.",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> None:
    """Получить маркер из OpenCV и сохранить готовую PNG-текстуру."""

    args = parse_args()
    dictionary_id = getattr(cv2.aruco, args.dictionary, None)
    if dictionary_id is None:
        raise ValueError(f"OpenCV не содержит словарь {args.dictionary!r}")

    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    marker_count = len(dictionary.bytesList)
    if not 0 <= args.marker_id < marker_count:
        raise ValueError(
            f"id должен находиться в диапазоне 0..{marker_count - 1}"
        )
    if args.pixels <= 0 or args.margin < 0:
        raise ValueError("pixels должен быть положительным, margin — неотрицательным")

    # В OpenCV 4.7+ используется generateImageMarker, а в Ubuntu 24.04
    # доступно прежнее имя drawMarker. Оба API используют тот же словарь.
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(
            dictionary, args.marker_id, args.pixels
        )
    else:
        marker = cv2.aruco.drawMarker(dictionary, args.marker_id, args.pixels)

    texture = cv2.copyMakeBorder(
        marker,
        args.margin,
        args.margin,
        args.margin,
        args.margin,
        cv2.BORDER_CONSTANT,
        value=255,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), texture):
        raise RuntimeError(f"Не удалось записать {args.output}")

    print(
        f"Создан {args.dictionary}, id={args.marker_id}: "
        f"{args.output} ({texture.shape[1]}x{texture.shape[0]})"
    )


if __name__ == "__main__":
    main()
