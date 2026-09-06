#!/usr/bin/env python3
"""Перевод выгрузки SAMannot в датасет YOLO для детекции.

Что делает и почему именно так
------------------------------

SAMannot отдаёт сегментацию: строки вида ``class x1 y1 x2 y2 ...`` с
десятками вершин. Для детекции нужны рамки, и переход к ним - это не
потеря качества, а смена представления: описанный прямоугольник хорошей
маски плотнее и стабильнее нарисованного рукой.

Три вещи, которые приходится делать по дороге.

Слияние вложенных контуров. Маска круглого знака распадается на несколько
контуров - внешний диск, внутренний, стрелка, - и каждый выгружается
отдельной строкой. Одна рамка на контур дала бы три рамки на один знак.
Поэтому пересекающиеся рамки сливаются, а разнесённые остаются
раздельными: два знака в кадре слипнуться не должны.

Отсев ошмётков. В выгрузке встречаются полигоны площадью в единицы
пикселей - следы неточной маски. Они превращаются в ложные рамки, и
модель училась бы находить шум.

Кадрирование. Съёмка велась вертикальным кадром телефона, а камера
робота даёт 4:3. Обучать на смеси нельзя: YOLO приводит всё к квадрату,
и вертикальный кадр сжимается по горизонтали втрое сильнее. Поэтому
кадры режутся и масштабируются, а координаты полигонов пересчитываются
тем же преобразованием - размечать заново не нужно.

Окно кадрирования по умолчанию выбирается по самой разметке: берётся
полоса нужного соотношения, содержащая все размеченные объекты с
запасом. Так знак гарантированно не окажется срезанным.
"""

from __future__ import annotations

import argparse
import random
import shutil
import sys
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    raise SystemExit("нужен Pillow: pip install pillow")


def read_polygons(path: Path) -> list[tuple[int, list[float], list[float]]]:
    """Прочитать полигоны YOLO-seg: класс и нормированные координаты."""

    polygons = []
    for line in path.read_text().splitlines():
        parts = line.split()
        # Меньше трёх вершин - не полигон.
        if len(parts) < 7:
            continue
        class_id = int(parts[0])
        values = [float(v) for v in parts[1:]]
        polygons.append((class_id, values[0::2], values[1::2]))
    return polygons


def boxes_overlap(a, b) -> bool:
    """Пересекаются ли рамки. Касание границами пересечением не считаем."""

    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def merge_boxes(boxes: list[list[float]]) -> list[list[float]]:
    """Слить пересекающиеся рамки в одну.

    Повторяется до тех пор, пока есть что сливать: три вложенных контура
    одного знака могут пересекаться попарно не все сразу.
    """

    merged = [list(b) for b in boxes]
    changed = True
    while changed:
        changed = False
        for i in range(len(merged)):
            for j in range(i + 1, len(merged)):
                if boxes_overlap(merged[i], merged[j]):
                    merged[i] = [
                        min(merged[i][0], merged[j][0]),
                        min(merged[i][1], merged[j][1]),
                        max(merged[i][2], merged[j][2]),
                        max(merged[i][3], merged[j][3]),
                    ]
                    merged.pop(j)
                    changed = True
                    break
            if changed:
                break
    return merged


def annotation_extent(label_files: list[Path]) -> tuple[float, float]:
    """Границы всей разметки по вертикали, в долях кадра."""

    top, bottom = 1.0, 0.0
    for path in label_files:
        for _, _, ys in read_polygons(path):
            top = min(top, min(ys))
            bottom = max(bottom, max(ys))
    return top, bottom


def pick_crop(top: float, bottom: float, width: int, height: int,
              aspect: float) -> tuple[int, int]:
    """Выбрать окно кадрирования, содержащее всю разметку.

    Берётся полная ширина; высота задаётся соотношением сторон целевого
    кадра. Окно центрируется на разметке и прижимается к краям, если
    вылезает за кадр.
    """

    crop_h = min(height, int(round(width / aspect)))
    centre = (top + bottom) / 2.0 * height
    offset = int(round(centre - crop_h / 2.0))
    offset = max(0, min(offset, height - crop_h))
    return crop_h, offset


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Собрать датасет YOLO-детекции из выгрузки SAMannot"
    )
    parser.add_argument("--source", required=True, type=Path,
                        help="каталог export/<сессия> из SAMannot")
    parser.add_argument("--out", required=True, type=Path,
                        help="куда сложить датасет")
    parser.add_argument("--width", type=int, default=640,
                        help="ширина кадра камеры робота")
    parser.add_argument("--height", type=int, default=480,
                        help="высота кадра камеры робота")
    parser.add_argument("--min-area", type=float, default=2000.0,
                        help="площадь рамки в пикселях исходника, ниже "
                             "которой полигон считается ошмётком маски")
    parser.add_argument("--val-share", type=float, default=0.2,
                        help="доля кадров в проверочной выборке")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-crop", action="store_true",
                        help="не кадрировать, только перевести в рамки")
    args = parser.parse_args()

    frames_dir = args.source / "frames"
    labels_dir = args.source / "labels"
    classes_file = args.source / "classes.txt"

    for path in (frames_dir, labels_dir, classes_file):
        if not path.exists():
            raise SystemExit(f"нет {path}")

    class_names = [
        line.strip()
        for line in classes_file.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    print(f"классы: {class_names}")

    label_files = sorted(labels_dir.glob("*.txt"))
    if not label_files:
        raise SystemExit("меток нет")

    sample = Image.open(sorted(frames_dir.glob("*"))[0])
    src_w, src_h = sample.size
    print(f"исходный кадр: {src_w}x{src_h}, кадров {len(label_files)}")

    if args.no_crop:
        crop_h, crop_y = src_h, 0
    else:
        top, bottom = annotation_extent(label_files)
        crop_h, crop_y = pick_crop(top, bottom, src_w, src_h,
                                   args.width / args.height)
        print(f"разметка по вертикали: {top * src_h:.0f}..{bottom * src_h:.0f}")
        print(f"окно кадрирования: {src_w}x{crop_h} со смещением y={crop_y}")

    for split in ("train", "val"):
        for kind in ("images", "labels"):
            (args.out / kind / split).mkdir(parents=True, exist_ok=True)

    random.seed(args.seed)
    stems = [p.stem for p in label_files]
    random.shuffle(stems)
    val_count = int(len(stems) * args.val_share)
    split_of = {s: ("val" if i < val_count else "train")
                for i, s in enumerate(stems)}

    kept = dropped_small = dropped_outside = merged_away = 0
    empty_frames = 0

    for label_path in label_files:
        stem = label_path.stem
        frame_path = next(iter(frames_dir.glob(stem + ".*")), None)
        if frame_path is None:
            continue

        by_class: dict[int, list[list[float]]] = {}
        for class_id, xs, ys in read_polygons(label_path):
            x0, x1 = min(xs) * src_w, max(xs) * src_w
            y0, y1 = min(ys) * src_h, max(ys) * src_h
            if (x1 - x0) * (y1 - y0) < args.min_area:
                dropped_small += 1
                continue
            by_class.setdefault(class_id, []).append([x0, y0, x1, y1])

        lines = []
        for class_id, boxes in by_class.items():
            before = len(boxes)
            for x0, y0, x1, y1 in merge_boxes(boxes):
                merged_away += before - 1 if before > 1 else 0
                before = 1

                # Пересчёт в систему обрезанного кадра.
                y0c, y1c = y0 - crop_y, y1 - crop_y
                if y1c <= 0 or y0c >= crop_h:
                    dropped_outside += 1
                    continue
                y0c, y1c = max(0.0, y0c), min(float(crop_h), y1c)

                cx = (x0 + x1) / 2.0 / src_w
                cy = (y0c + y1c) / 2.0 / crop_h
                bw = (x1 - x0) / src_w
                bh = (y1c - y0c) / crop_h
                lines.append(f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
                kept += 1

        if not lines:
            empty_frames += 1

        split = split_of[stem]
        (args.out / "labels" / split / f"{stem}.txt").write_text(
            "\n".join(lines) + ("\n" if lines else "")
        )

        image = Image.open(frame_path).convert("RGB")
        if not args.no_crop:
            image = image.crop((0, crop_y, src_w, crop_y + crop_h))
        image = image.resize((args.width, args.height), Image.LANCZOS)
        image.save(args.out / "images" / split / f"{stem}.jpg", quality=95)

    data_yaml = args.out / "data.yaml"
    data_yaml.write_text(
        "# Собрано samannot_to_yolo.py из выгрузки SAMannot.\n"
        "#\n"
        "# Порядок классов обязан совпадать с class_id_to_label в\n"
        "# src/rtk2026_cv/config/onnx_sign_detector.yaml: детектор читает\n"
        "# идентификатор по позиции, и перестановка молча сломает\n"
        "# сопоставление знака с командой маршрута.\n"
        f"path: {args.out.resolve()}\n"
        "train: images/train\n"
        "val: images/val\n"
        f"nc: {len(class_names)}\n"
        f"names: {class_names}\n",
        encoding="utf-8",
    )

    print()
    print(f"рамок оставлено      : {kept}")
    print(f"слито вложенных      : {merged_away}")
    print(f"отсеяно мелких       : {dropped_small}")
    print(f"вне окна кадрирования: {dropped_outside}")
    print(f"кадров без разметки  : {empty_frames}")
    print(f"train / val          : "
          f"{len(stems) - val_count} / {val_count}")
    print(f"\n{data_yaml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
