#!/usr/bin/env python3
"""
Detector trainer — trains the on-robot ball/flower YOLO and exports TFLite.

Heavy training happens HERE on your computer (the teacher); the robot only
ever runs the distilled student. Exports a float-I/O TFLite model (dynamic
range quantization) that the robot's DetectorProcessor loads directly.

Usage:
    python3 train_detector.py dataset
    python3 train_detector.py dataset --model yolov8n.pt --epochs 100 --imgsz 320

Requirements (laptop only, never on the robot):
    pip install ultralytics

Notes:
  * --model accepts ANY ultralytics nano/small model — use the newest tiny
    YOLO your installed ultralytics supports (e.g. a yolo26n/yolo11n/yolov8n
    .pt). Keep imgsz small (320): the REV Control Hub must run it in real time
    next to the rest of the vision stack.
  * Class ids MUST match NN_LABELS in globals.kt:
    0 Pollen, 1 Nectar Blue, 2 Nectar Red, 3 Flower.
"""

import argparse
import os
import sys

CLASSES = ["Pollen", "Nectar Blue", "Nectar Red", "Flower"]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", help="dataset folder from auto_label.py (+ review)")
    ap.add_argument("--model", default="yolov8n.pt", help="base model (any tiny ultralytics YOLO)")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=320, help="input size — keep 320 for the hub CPU")
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--project", default="runs/biobuzz")
    args = ap.parse_args()

    img_train = os.path.join(args.dataset, "images", "train")
    img_val = os.path.join(args.dataset, "images", "val")
    if not os.path.isdir(img_train) or not os.path.isdir(img_val):
        sys.exit("! expected {}/images/{{train,val}} — run auto_label.py first".format(args.dataset))

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("Training needs the ultralytics package (laptop only):  "
                 "pip install ultralytics")

    yaml_path = os.path.join(args.dataset, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write("path: {}\n".format(os.path.abspath(args.dataset)))
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("nc: {}\n".format(len(CLASSES)))
        f.write("names: [{}]\n".format(", ".join(CLASSES)))
    print("* dataset yaml written to", yaml_path)

    model = YOLO(args.model)
    model.train(
        data=yaml_path,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.project,
        name="ball_detector",
        patience=25,
    )

    print("\n* training done — exporting TFLite (float I/O, dynamic range quant)...")
    best = os.path.join(args.project, "ball_detector", "weights", "best.pt")
    tflite_path = YOLO(best).export(format="tflite")
    print("\n==== DEPLOY ====")
    print("1. copy the exported model:")
    print("      {}".format(tflite_path))
    print("   to the RC app's external storage as:")
    print("      nn/ball_detector.tflite   (same folder level as blackbox/)")
    print("2. Confirm NN_DETECTOR_MODE in globals.kt (start at ADVISORY!).")
    print("3. Run the 'Neural Net Test' OpMode on the bench: boxes appear cyan")
    print("   on the DS preview; check the referee isn't hallucinating before")
    print("   flipping to REFEREE.")
    print("4. On the hub CPU keep NN_DETECT_HZ low (3 is a good start).")


if __name__ == "__main__":
    main()