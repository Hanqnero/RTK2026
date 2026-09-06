#!/usr/bin/env python3
"""Дообучение головы детектора: исправление путаницы левого и правого.

Что чинится
-----------

В матрице ошибок прошлого обучения истинный turn_right распознавался как
turn_left в 86 % случаев, а верно - в 6 %. Остальные шесть классов при
этом держали 0.94-0.98.

Причина в настройках того запуска: fliplr: 0.5. Половина кадров
отражалась по горизонтали, а метка оставалась прежней - модели показывали
правый знак и называли его левым. Симметричные классы (bus_stop,
parking_spot, obstacle) от этого не пострадали, направленные - полностью.

Почему хватает головы
---------------------

Ошибка классификационная, а свёрточная основа зеркально-инвариантной не
является: направление стрелки она кодирует. Игнорировать это направление
научили голову. Поэтому основа замораживается, а переучивается только
голова - это и быстрее, и бьёт в причину.

freeze: 10 у yolo26n соответствует основе; голова и шея остаются
обучаемыми.

Обязательное условие
--------------------

fliplr: 0.0. Отражение как аугментация уничтожает ровно тот признак,
ради которого дообучение и затевается. Остальная аугментация безопасна:
поворот, масштаб, сдвиг и цвет направление не меняют.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Дообучить голову детектора знаков"
    )
    parser.add_argument("--weights", type=Path,
                        default=Path("runs/detect/runs/yolo26/fast_best-2/weights/best.pt"),
                        help="исходные веса")
    parser.add_argument("--data", type=Path, default=Path("dataset_signs/data.yaml"))
    parser.add_argument("--epochs", type=int, default=3,
                        help="для пробного прогона хватает трёх")
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="mps",
                        help="mps на Apple Silicon, cpu для сверки, 0 для CUDA")
    parser.add_argument("--freeze", type=int, default=10,
                        help="сколько слоёв основы заморозить; 0 - учить всё")
    parser.add_argument("--lr0", type=float, default=0.0005,
                        help="меньше обычного: голова дообучается, а не "
                             "учится с нуля")
    parser.add_argument("--name", default="head_finetune")
    args = parser.parse_args()

    here = Path(__file__).resolve().parent
    weights = (here / args.weights).resolve() if not args.weights.is_absolute() else args.weights
    data = (here / args.data).resolve() if not args.data.is_absolute() else args.data

    for path in (weights, data):
        if not path.exists():
            raise SystemExit(f"нет {path}")

    print(f"веса : {weights}")
    print(f"данные: {data}")
    print(f"заморожено слоёв: {args.freeze}, lr0={args.lr0}, "
          f"эпох={args.epochs}, устройство={args.device}")
    print()

    model = YOLO(str(weights))

    model.train(
        data=str(data),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        device=args.device,
        freeze=args.freeze,
        lr0=args.lr0,
        optimizer="AdamW",

        # ~ Аугментация.
        #
        # fliplr строго ноль - см. пояснение в начале файла. Это не
        # настройка вкуса, а условие корректности обучения.
        fliplr=0.0,
        flipud=0.0,

        # Мозаика склеивает четыре кадра в один. На маленьком наборе она
        # чаще мешает, чем помогает: знак попадает на границу склейки и
        # обрезается, а таких кадров в реальности не бывает.
        mosaic=0.0,

        # Безопасное: направление знака не меняют.
        degrees=10.0,
        scale=0.4,
        shear=2.0,
        hsv_h=0.015,
        hsv_s=0.6,
        hsv_v=0.4,

        project=str(here / "runs" / "finetune"),
        name=args.name,
        exist_ok=True,
        plots=True,
        val=True,
    )

    print("\nготово")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
