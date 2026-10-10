#!/usr/bin/env python3
"""
Automatic flywheel calibration analyzer for the 16660 BioBuzz robot.

Tracks the ball in a calibration video (filmed with the "Flywheel Calibration"
OpMode running) and measures how far each shot lands — no manual measuring.
The output is formatted to paste straight into globals.kt
(SHOOTER_CALIBRATION_DISTANCE_CM).

SETUP (see tools/README.md for the full walkthrough):
  * camera FIXED and roughly SIDE-ON to the ball's flight path, so "lowest
    point in the frame" = "ball on the floor"
  * two tape marks on the floor a known distance apart, INSIDE the camera
    view, roughly along the flight direction (the scale reference)
  * film the shots in the SAME order as --rpm (default 6000 down to 1000),
    one after another in a single video (or run once per video with one --rpm)

Usage:
    python3 flywheel_analyzer.py calib.mp4
    python3 flywheel_analyzer.py calib.mp4 --rpm 6000 5000 4000 3000 2000 1000
    python3 flywheel_analyzer.py shot1.mp4 --rpm 6000 --hsv 14 100 100 35 255 255
    python3 flywheel_analyzer.py calib.mp4 --preset hsv_presets.json --preset-name POLLEN_HSV_RANGE
"""

import argparse
import json
import math
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("This tool needs OpenCV + NumPy. Run:  pip install -r tools/requirements.txt")

WIN = "Flywheel Analyzer"


def dist_px(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def parse_hsv(text):
    vals = [float(x) for x in text.split()]
    if len(vals) != 6:
        sys.exit("--hsv needs exactly 6 numbers: H_MIN S_MIN V_MIN H_MAX S_MAX V_MAX")
    return np.array(vals[:3], np.uint8), np.array(vals[3:], np.uint8)


def hsv_from_preset(path, key):
    with open(path) as f:
        data = json.load(f)
    if key is None:
        key = next(iter(data))
    p = data[key]
    lo = np.array([p["H min"], p["S min"], p["V min"]], np.uint8)
    hi = np.array([p["H max"], p["S max"], p["V max"]], np.uint8)
    print("* ball HSV from preset '{}' of {}".format(key, path))
    return lo, hi

# ---- stage 1: click calibration (launch point + scale reference) -----------------
clicks = []


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicks.append((x, y))


def collect_calibration(frame, ref_cm):
    """Shows the first frame; returns (launch, cm_per_px)."""
    global clicks
    clicks = []
    while True:
        disp = frame.copy()
        cv2.putText(disp, "1) click the LAUNCH point (where the ball exits the flywheel)",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(disp, "2&3) click the TWO tape marks {:.0f} cm apart".format(ref_cm),
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(disp, "u undo | c confirm | ESC quit", (10, 76),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        for i, p in enumerate(clicks):
            cv2.circle(disp, p, 6, (0, 0, 255), 2)
            cv2.putText(disp, str(i + 1), (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if len(clicks) >= 3:
            cv2.line(disp, clicks[1], clicks[2], (255, 0, 0), 2)
        cv2.imshow(WIN, disp)
        key = cv2.waitKey(30) & 0xFF
        if key == 27:
            sys.exit("cancelled")
        elif key == ord('u') and clicks:
            clicks.pop()
        elif key == ord('c') and len(clicks) >= 3:
            break
    launch, ref_a, ref_b = clicks[0], clicks[1], clicks[2]
    cm_per_px = ref_cm / max(dist_px(ref_a, ref_b), 1e-9)
    print("* scale: {:.4f} cm/px (launch {}, ref pair {:.1f} px apart)".format(
        cm_per_px, launch, dist_px(ref_a, ref_b)))
    return launch, cm_per_px


def detect_ball(frame_hsv, lo, hi, min_area):
    """Largest HSV blob centroid, or None."""
    mask = cv2.inRange(frame_hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, min_area
    for c in contours:
        a = cv2.contourArea(c)
        if a >= best_area:
            m = cv2.moments(c)
            if m["m00"] > 0:
                best = (m["m10"] / m["m00"], m["m01"] / m["m00"])
                best_area = a
    return best

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", help="calibration video")
    ap.add_argument("--rpm", nargs="+", type=float, default=[6000, 5000, 4000, 3000, 2000, 1000],
                    help="RPM of each shot, IN THE ORDER FILMED")
    ap.add_argument("--hsv", help="ball HSV: 'H_MIN S_MIN V_MIN H_MAX S_MAX V_MAX' (tune with hsv_calibrator.py)")
    ap.add_argument("--preset", help="JSON preset file saved by hsv_calibrator.py")
    ap.add_argument("--preset-name", default=None)
    ap.add_argument("--ref-cm", type=float, default=100.0, help="distance between the two tape marks")
    ap.add_argument("--min-area", type=float, default=80.0, help="min ball blob area (px^2)")
    ap.add_argument("--launch-radius", type=float, default=120.0,
                    help="px around the launch point where a new shot may start")
    ap.add_argument("--lost-frames", type=int, default=25, help="frames without the ball that end a track")
    ap.add_argument("--no-show", action="store_true", help="don't play the annotated video")
    ap.add_argument("--json", default=None, help="results JSON path (default: <video>_calibration.json)")
    args = ap.parse_args()

    if args.hsv:
        lo, hi = parse_hsv(args.hsv)
    elif args.preset and os.path.exists(args.preset):
        lo, hi = hsv_from_preset(args.preset, args.preset_name)
    else:
        print("! no --hsv or --preset given; assuming Pollen yellow (tune with hsv_calibrator.py)")
        lo, hi = np.array([14, 100, 100], np.uint8), np.array([35, 255, 255], np.uint8)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit("cannot open video: {}".format(args.video))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    ok, first = cap.read()
    if not ok:
        sys.exit("empty video")
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse)
    launch, cm_per_px = collect_calibration(first, args.ref_cm)
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    tracks, current = [], None
    miss = 0
    show = not args.no_show

    def finalize(track):
        pts = track["points"]
        if len(pts) < 5:
            print("! ignoring a {}-point blip near launch".format(len(pts)))
            return
        # landing = lowest point in the frame during the tracked flight
        # (camera is side-on, so the ball's lowest position = floor contact)
        landing = max(pts, key=lambda p: p[1])
        d_cm = dist_px(landing, launch) * cm_per_px
        tracks.append({"landing": landing, "distance_cm": d_cm})
        print("* shot #{}: {} tracked pts, landed {:.1f} cm from launch".format(
            len(tracks), len(pts), d_cm))

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        det = detect_ball(hsv, lo, hi, args.min_area)

        if current is None:
            # a new shot begins when the ball shows up at the launch point
            if det and dist_px(det, launch) <= args.launch_radius:
                current = {"points": [det]}
                miss = 0
        else:
            if det:
                near_launch = dist_px(det, launch) <= args.launch_radius
                last_far = dist_px(current["points"][-1], launch) > args.launch_radius
                # ball back at the launcher = flight over, next shot begins
                if len(current["points"]) >= 10 and near_launch and last_far:
                    finalize(current)
                    current = {"points": [det]}
                    miss = 0
                else:
                    current["points"].append(det)
                    miss = 0
            else:
                miss += 1
                if miss > args.lost_frames:
                    finalize(current)
                    current = None

        if show:
            disp = frame.copy()
            cv2.circle(disp, (int(launch[0]), int(launch[1])), 8, (0, 0, 255), 2)
            if current:
                for p in current["points"]:
                    cv2.circle(disp, (int(p[0]), int(p[1])), 3, (0, 255, 0), -1)
            for t in tracks:
                cv2.drawMarker(disp, (int(t["landing"][0]), int(t["landing"][1])),
                               (255, 0, 0), cv2.MARKER_TILTED_CROSS, 18, 2)
            cv2.putText(disp, "tracks: {} | q = fast-forward".format(len(tracks)),
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(WIN, disp)
            key = cv2.waitKey(max(1, int(1000 / fps))) & 0xFF
            if key in (ord('q'), 27):
                show = False

    if current:
        finalize(current)
    cap.release()
    cv2.destroyAllWindows()

    if not tracks:
        sys.exit("! no ball tracks found — retune HSV with hsv_calibrator.py, "
                 "lower --min-area, or raise --launch-radius")

    if len(tracks) != len(args.rpm):
        print("! WARNING: found {} shots but {} RPM values given".format(len(tracks), len(args.rpm)))

    # pair each filmed shot with its RPM, then order by RPM descending
    pairs = [(rpm, t["distance_cm"]) for rpm, t in zip(args.rpm, tracks)]
    pairs.sort(key=lambda p: -p[0])

    print("\n==== MEASURED SHOOTER CURVE ====")
    for rpm, d in pairs:
        print("  {:>6.0f} RPM -> {:>7.1f} cm".format(rpm, d))

    print("\n---- paste into globals.kt ----")
    print("val SHOOTER_CALIBRATION_RPM = doubleArrayOf({})".format(
        ", ".join("{:.1f}".format(r) for r, _ in pairs)))
    print("var SHOOTER_CALIBRATION_DISTANCE_CM = doubleArrayOf(")
    for rpm, d in pairs:
        print("    {:.1f}, // {:.0f} RPM".format(d, rpm))
    print(")")
    print("--------------------------------\n")

    out_path = args.json or (os.path.splitext(args.video)[0] + "_calibration.json")
    with open(out_path, "w") as f:
        json.dump({"rpm": [r for r, _ in pairs], "distance_cm": [d for _, d in pairs],
                   "cm_per_px": cm_per_px, "launch_px": list(launch)}, f, indent=2)
    print("* full results saved to {}".format(out_path))


if __name__ == "__main__":
    main()


