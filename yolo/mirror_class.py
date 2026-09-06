#!/usr/bin/env python3
"""Получить зеркальный класс знака отражением по горизонтали.

Зачем
-----

Знаки "направо" и "налево" зеркальны друг другу, поэтому отражённая
съёмка правого поворота даёт визуально верный левый. Это позволяет
проверить пайплайн, не дожидаясь второй съёмки.

Чем это отличается от аугментации fliplr
----------------------------------------

Ровно тем, что метка меняется вместе с картинкой. Прошлое обучение шло с
fliplr: 0.5, то есть половина кадров отражалась, а класс оставался
прежним - модели показывали правый знак и называли его левым. В матрице
ошибок это дало 0.86: почти каждый правый распознавался как левый.

Здесь наоборот: отражая кадр, мы меняем и класс. Направление становится
признаком, а не шумом.

Чего этим не добиться
---------------------

Отражается вся сцена, а не только знак: трава, разметка, фон. Модель
может связать класс с зеркальностью обстановки, а не со стрелкой -
особенно если оба класса взяты из одной съёмки. Для обкатки схемы это
приемлемо, для рабочего датасета левый поворот надо снять отдельно.

Идентификаторы классов
----------------------

Ставятся по src/rtk2026_cv/config/onnx_sign_detector.yaml, где детектор
читает класс по позиции. Пустые классы в списке остаются намеренно:
так обученная модель подключается к детектору без переназначения.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

try:
    from PIL import Image, ImageOps
except ImportError:
    raise SystemExit("нужен Pillow: pip install pillow")


#: Порядок обязан совпадать с class_id_to_label детектора.
CLASS_NAMES = [
    "bus_stop",
    "move_forward",
    "no_turn_left",
    "no_turn_right",
    "obstacle",
    "parking_spot",
    "turn_left",
    "turn_right",
]


def mirror_label(line: str, new_class_id: int) -> str:
    """Отразить рамку по горизонтали и сменить класс.

    В нормированных координатах YOLO отражение - это cx -> 1 - cx.
    Ширина, высота и cy не меняются.
    """

    parts = line.split()
    _, cx, cy, bw, bh = parts[0], *map(float, parts[1:5])
    return f"{new_class_id} {1.0 - cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Добавить зеркальный класс в датасет YOLO"
    )
    parser.add_argument("--source", required=True, type=Path,
                        help="датасет с исходным классом")
    parser.add_argument("--out", required=True, type=Path,
                        help="куда сложить объединённый датасет")
    parser.add_argument("--source-class", default="turn_right",
                        help="класс в исходном датасете")
    parser.add_argument("--mirror-class", default="turn_left",
                        help="класс, которым помечается отражённая копия")
    args = parser.parse_args()

    if args.source_class not in CLASS_NAMES:
        raise SystemExit(f"{args.source_class} нет в списке классов детектора")
    if args.mirror_class not in CLASS_NAMES:
        raise SystemExit(f"{args.mirror_class} нет в списке классов детектора")

    source_id = CLASS_NAMES.index(args.source_class)
    mirror_id = CLASS_NAMES.index(args.mirror_class)
    print(f"{args.source_class} -> {source_id}, "
          f"{args.mirror_class} -> {mirror_id} (зеркало)")

    copied = mirrored = 0

    for split in ("train", "val"):
        (args.out / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.out / "labels" / split).mkdir(parents=True, exist_ok=True)

        for label_path in sorted((args.source / "labels" / split).glob("*.txt")):
            stem = label_path.stem
            image_path = next(
                iter((args.source / "images" / split).glob(stem + ".*")), None
            )
            if image_path is None:
                continue

            lines = [l for l in label_path.read_text().splitlines() if l.strip()]

            # Исходный кадр: класс переносится в нумерацию детектора.
            renumbered = [
                f"{source_id} " + " ".join(l.split()[1:]) for l in lines
            ]
            (args.out / "labels" / split / f"{stem}.txt").write_text(
                "\n".join(renumbered) + "\n"
            )
            shutil.copy2(image_path,
                         args.out / "images" / split / image_path.name)
            copied += 1

            # Зеркальная копия под другим классом.
            image = Image.open(image_path).convert("RGB")
            ImageOps.mirror(image).save(
                args.out / "images" / split / f"{stem}_mirror.jpg", quality=95
            )
            (args.out / "labels" / split / f"{stem}_mirror.txt").write_text(
                "\n".join(mirror_label(l, mirror_id) for l in lines) + "\n"
            )
            mirrored += 1

    (args.out / "data.yaml").write_text(
        "# Собрано mirror_class.py.\n"
        "#\n"
        f"# {args.mirror_class} получен отражением {args.source_class}: знаки\n"
        "# зеркальны, поэтому отражённый кадр визуально верен. Отражается\n"
        "# при этом вся сцена, и модель может связать класс с зеркальностью\n"
        "# обстановки, а не со стрелкой. Годится для проверки схемы; для\n"
        "# рабочего датасета второй класс надо снять отдельно.\n"
        "#\n"
        "# ВАЖНО: обучать с fliplr: 0.0. Отражение как аугментация здесь\n"
        "# уничтожает именно тот признак, ради которого датасет и собран.\n"
        "#\n"
        "# Порядок классов совпадает с class_id_to_label в\n"
        "# src/rtk2026_cv/config/onnx_sign_detector.yaml: пустые классы\n"
        "# оставлены намеренно, чтобы модель подключалась без переназначения.\n"
        f"path: {args.out.resolve()}\n"
        "train: images/train\n"
        "val: images/val\n"
        f"nc: {len(CLASS_NAMES)}\n"
        f"names: {CLASS_NAMES}\n",
        encoding="utf-8",
    )

    print(f"исходных кадров : {copied}")
    print(f"зеркальных      : {mirrored}")
    print(f"всего           : {copied + mirrored}")
    print(f"\n{args.out / 'data.yaml'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
