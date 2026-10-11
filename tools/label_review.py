#!/usr/bin/env python3
"""
Label review — quickly fix the auto-labeler's mistakes.

Cycles through a YOLO dataset (images/ + labels/), shows every box, and lets
you fix the ~5% the HSV teacher got wrong:

    n / p      next / previous image
    a + drag   add a box (class = current; c cycles classes)
    click box  select it; d deletes the selected box
    s          save labels for this image
    u          unsaved changes: revert to last saved
    q          quit

Usage:
    python3 label_review.py dataset
"""

import argparse
import os
import sys
import glob

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("This tool needs OpenCV + NumPy. Run:  pip install -r tools/requirements.txt")

CLASSES = ["Pollen", "Nectar Blue", "Nectar Red", "Flower"]
CLASS_COLORS = [(0, 255, 255), (255, 120, 0), (0, 0, 255), (0, 255, 0)]


def load_labels(path):
    boxes = []
    if os.path.exists(path):
        with open(path) as f:
            for line in f:
                parts = line.split()
                if len(parts) != 5:
                    continue
                boxes.append([int(parts[0])] + [float(v) for v in parts[1:]])
    return boxes


def save_labels(path, boxes, w, h):
    with open(path, "w") as f:
        for b in boxes:
            cls, cx, cy, bw, bh = b[0], b[1], b[2], b[3], b[4]
            f.write("{} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(cls, cx, cy, bw, bh))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", help="dataset folder made by auto_label.py")
    args = ap.parse_args()

    images = sorted(
        glob.glob(os.path.join(args.dataset, "images", "*", "*.jpg")) +
        glob.glob(os.path.join(args.dataset, "images", "*", "*.png")))
    if not images:
        sys.exit("! no images under {}/images/".format(args.dataset))

    def label_path(img_path):
        split = os.path.basename(os.path.dirname(img_path))
        return os.path.join(args.dataset, "labels", split,
                            os.path.splitext(os.path.basename(img_path))[0] + ".txt")

    i = 0
    cur_cls = 0
    state = {
        "boxes": [], "img_w": 0, "img_h": 0,
        "adding": False, "drag_start": None, "drag_end": None,
        "selected": None, "dirty": False,
    }

    def pt_in_box(px, py, b, w, h):
        cx, cy, bw, bh = b[1] * w, b[2] * h, b[3] * w, b[4] * h
        return abs(px - cx) <= bw / 2 and abs(py - cy) <= bh / 2

    def on_mouse(event, x, y, flags, param):
        s = state
        if event == cv2.EVENT_LBUTTONDOWN:
            if s["adding"]:
                s["drag_start"] = (x, y)
                s["drag_end"] = (x, y)
            else:
                # select the box under the cursor (smallest one wins)
                best, best_area = None, 1e18
                for bi, b in enumerate(s["boxes"]):
                    if pt_in_box(x, y, b, s["img_w"], s["img_h"]):
                        a = b[3] * b[4]
                        if a < best_area:
                            best, best_area = bi, a
                s["selected"] = best
        elif event == cv2.EVENT_MOUSEMOVE and s["drag_start"] is not None:
            s["drag_end"] = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and s["drag_start"] is not None:
            x1, y1 = s["drag_start"]
            x2, y2 = s["drag_end"] = (x, y)
            s["drag_start"] = None
            if abs(x2 - x1) > 6 and abs(y2 - y1) > 6:
                bx = min(x1, x2) / s["img_w"]
                by = min(y1, y2) / s["img_h"]
                s["boxes"].append([cur_cls,
                                   bx + abs(x2 - x1) / 2 / s["img_w"],
                                   by + abs(y2 - y1) / 2 / s["img_h"],
                                   abs(x2 - x1) / s["img_w"],
                                   abs(y2 - y1) / s["img_h"]])
                s["dirty"] = True

    cv2.namedWindow("label review")
    cv2.setMouseCallback("label review", on_mouse)

    print("keys: n/p next/prev | a toggle add-mode (drag to draw) | click select | "
          "d delete selected | c cycle add-class | s save | u revert | q quit")

    while True:
        img_path = images[i]
        img = cv2.imread(img_path)
        if img is None:
            i = (i + 1) % len(images)
            continue
        h, w = img.shape[:2]
        if not state["dirty"]:
            state["boxes"] = load_labels(label_path(img_path))
            state["selected"] = None
        state["img_w"], state["img_h"] = w, h

        disp = img.copy()
        for bi, b in enumerate(state["boxes"]):
            cx, cy, bw, bh = b[1] * w, b[2] * h, b[3] * w, b[4] * h
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            color = CLASS_COLORS[b[0] % len(CLASS_COLORS)]
            if bi == state["selected"]:
                color = (255, 255, 255)
            cv2.rectangle(disp, (x1, y1), (x2, y2), color, 2)
            cv2.putText(disp, CLASSES[b[0]], (x1, max(14, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if state["drag_start"] and state["drag_end"]:
            cv2.rectangle(disp, state["drag_start"], state["drag_end"],
                          CLASS_COLORS[cur_cls], 2)

        mode = "ADD {}".format(CLASSES[cur_cls]) if state["adding"] else "VIEW"
        status = "[{}/{}] {} boxes {} | {} | {}".format(
            i + 1, len(images), len(state["boxes"]),
            "DIRTY (s=save)" if state["dirty"] else "saved",
            mode, os.path.basename(img_path))
        cv2.putText(disp, status, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 2)
        cv2.imshow("label review", disp)
        key = cv2.waitKey(30) & 0xFF

        if key in (ord('q'), 27):
            break
        elif key == ord('n'):
            i = (i + 1) % len(images)
            state["dirty"] = False
        elif key == ord('p'):
            i = (i - 1) % len(images)
            state["dirty"] = False
        elif key == ord('a'):
            state["adding"] = not state["adding"]
        elif key == ord('c'):
            cur_cls = (cur_cls + 1) % len(CLASSES)
        elif key == ord('d'):
            if state["selected"] is not None:
                del state["boxes"][state["selected"]]
                state["selected"] = None
                state["dirty"] = True
        elif key == ord('s'):
            save_labels(label_path(img_path), state["boxes"], w, h)
            state["dirty"] = False
        elif key == ord('u'):
            state["dirty"] = False

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
