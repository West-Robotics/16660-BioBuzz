#!/usr/bin/env python3
"""
HSV color calibrator for the 16660 BioBuzz robot.

Live GUI (OpenCV) for tuning the HSV ranges used by the robot's vision code:
  * sliders for H/S/V min & max (OpenCV hue scale 0-180 — the SAME scale the
    robot's Kotlin ColorRange uses, so values paste over directly)
  * live mask preview + result preview
  * automatic contour detection with bounding boxes drawn on the frame
  * drag a bounding box with the mouse, then press 'f' to FIT the sliders to
    the colors inside that box
  * prints paste-ready Kotlin ColorRange code ('s') and saves JSON presets ('p')
    that flywheel_analyzer.py can load

Usage:
    python3 hsv_calibrator.py --camera 0                     # live webcam
    python3 hsv_calibrator.py field_video.mp4                # recorded video
    python3 hsv_calibrator.py video.mp4 --name POLLEN_HSV_RANGE
    python3 hsv_calibrator.py --camera 0 --preset hsv_presets.json

Keys (in the "Frame" window):
    SPACE   pause / resume video          f   fit sliders to last bounding box
    s       print Kotlin ColorRange       p   save preset to JSON
    u       undo last bounding box        q   quit
"""

import argparse
import json
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("This tool needs OpenCV + NumPy. Run:  pip install -r tools/requirements.txt")

WIN, MASK_WIN, RESULT_WIN, CTRL_WIN = "Frame", "Mask", "Result", "HSV Controls"
BAR_NAMES = ("H min", "H max", "S min", "S max", "V min", "V max")


def nothing(_):
    pass


def build_trackbars(init):
    cv2.namedWindow(CTRL_WIN)
    limits = (180, 180, 255, 255, 255, 255)
    for name, mx, val in zip(BAR_NAMES, limits, init):
        cv2.createTrackbar(name, CTRL_WIN, int(val), mx, nothing)


def read_trackbars():
    g = lambda n: cv2.getTrackbarPos(n, CTRL_WIN)
    lo = np.array([g("H min"), g("S min"), g("V min")], np.uint8)
    hi = np.array([g("H max"), g("S max"), g("V max")], np.uint8)
    return lo, hi


def fit_trackbars(lo, hi):
    for name, val in zip(BAR_NAMES, list(lo) + list(hi)):
        cv2.setTrackbarPos(name, CTRL_WIN, int(val))


def kotlin_snippet(name, lo, hi):
    return (
        "val {} = ColorRange(\n".format(name or "MY_HSV_RANGE") +
        "    ColorSpace.HSV,\n" +
        "    Scalar({}, {}, {}),\n".format(float(lo[0]), float(lo[1]), float(lo[2])) +
        "    Scalar({}, {}, {})\n".format(float(hi[0]), float(hi[1]), float(hi[2])) +
        ")"
    )


def save_preset(path, key, lo, hi):
    data = {}
    if os.path.exists(path):
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError):
            print("! could not read existing preset file, starting fresh")
    if not key:
        key = "range_{}".format(len(data))
    data[key] = {n: int(v) for n, v in zip(BAR_NAMES, list(lo) + list(hi))}
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print('* saved preset "{}" to {}'.format(key, path))


def load_preset(path, key):
    with open(path) as f:
        data = json.load(f)
    if key is None:
        key = next(iter(data))
    p = data[key]
    print('* loaded preset "{}" from {}'.format(key, path))
    return tuple(p[n] for n in BAR_NAMES)

# ---- mouse bounding-box dragging -------------------------------------------------
drag = {"start": None, "box": None, "last": None, "stats": None}


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        drag["start"] = (x, y)
        drag["box"] = (x, y, x, y)
    elif event == cv2.EVENT_MOUSEMOVE and drag["start"] is not None:
        sx, sy = drag["start"]
        drag["box"] = (min(sx, x), min(sy, y), max(sx, x), max(sy, y))
    elif event == cv2.EVENT_LBUTTONUP and drag["start"] is not None:
        sx, sy = drag["start"]
        drag["box"] = (min(sx, x), min(sy, y), max(sx, x), max(sy, y))
        drag["start"] = None
        drag["last"] = drag["box"]


def box_stats(frame_hsv, box):
    x1, y1, x2, y2 = [int(v) for v in box]
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None
    roi = frame_hsv[y1:y2, x1:x2]
    lo = roi.reshape(-1, 3).min(axis=0)
    hi = roi.reshape(-1, 3).max(axis=0)
    return lo, hi


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("video", nargs="?", help="video file to calibrate on")
    src.add_argument("--camera", type=int, help="live webcam index (e.g. 0)")
    ap.add_argument("--name", default="MY_HSV_RANGE", help="name for the Kotlin snippet / preset key")
    ap.add_argument("--preset", default="hsv_presets.json", help="JSON preset file to load/save")
    ap.add_argument("--load-key", default=None, help="which preset key to load")
    ap.add_argument("--min-area", type=float, default=150.0, help="min contour area (px^2) to count as an object")
    args = ap.parse_args()

    if args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            sys.exit("cannot open camera {}".format(args.camera))
        is_camera = True
    else:
        if not os.path.exists(args.video):
            sys.exit("file not found: {}".format(args.video))
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            sys.exit("cannot open video: {}".format(args.video))
        is_camera = False

    init = load_preset(args.preset, args.load_key) if os.path.exists(args.preset) \
        else (14, 35, 100, 255, 100, 255)
    build_trackbars(init)
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse)

    paused, frame = False, None
    print("keys: SPACE pause | f fit to box | s print Kotlin | p save preset | u undo box | q quit")
    while True:
        if not paused or frame is None:
            ok, frame = cap.read()
            if not ok:
                if is_camera:
                    break
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop the video
                continue
            if frame.shape[1] > 1280:
                scale = 1280.0 / frame.shape[1]
                frame = cv2.resize(frame, (1280, int(frame.shape[0] * scale)))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo, hi = read_trackbars()
        mask = cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # automatic detection with bounding boxes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= args.min_area]
        display = frame.copy()
        for i, c in enumerate(sorted(contours, key=cv2.contourArea, reverse=True)):
            x, y, w, h = cv2.boundingRect(c)
            color = (0, 255, 0) if i == 0 else (0, 200, 255)
            cv2.rectangle(display, (x, y), (x + w, y + h), color, 2)
            cv2.putText(display, "{}px2".format(int(cv2.contourArea(c))), (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # bounding box being dragged / last one, plus its HSV stats
        box = drag["box"] if drag["start"] is not None else drag["last"]
        if box is not None:
            x1, y1, x2, y2 = box
            cv2.rectangle(display, (x1, y1), (x2, y2), (255, 60, 60), 2)
            stats = box_stats(hsv, box)
            if stats:
                drag["stats"] = stats
                cv2.putText(display, "box: press f to fit sliders", (x1, max(y1 - 22, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 60, 60), 1)

        cv2.imshow(WIN, display)
        cv2.imshow(MASK_WIN, mask)
        cv2.imshow(RESULT_WIN, result)

        key = cv2.waitKey(30 if paused else 1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord(' '):
            paused = not paused
        elif key == ord('u'):
            drag["last"], drag["stats"] = None, None
        elif key == ord('f') and drag["stats"]:
            lo_s, hi_s = drag["stats"]
            fit_trackbars(lo_s, hi_s)
            print("* sliders fitted to box: lo={} hi={}".format(list(lo_s), list(hi_s)))
        elif key == ord('s'):
            print("\n---- paste into globals.kt (import ColorRange/ColorSpace/Scalar) ----")
            print(kotlin_snippet(args.name, lo, hi))
            print("--------------------------------------------------------------------\n")
        elif key == ord('p'):
            save_preset(args.preset, args.name, lo, hi)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
