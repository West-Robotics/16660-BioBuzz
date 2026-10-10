#!/usr/bin/env python3
"""
Black Box dataset builder — turns a recorded match into a computer-friendly dataset.

The robot's Black Box (TeamCode/.../blackbox/BlackBox.kt) records every match
as a session folder on the Robot Controller:

    blackbox/<label>_<timestamp>/
        state.jsonl          one JSON line per record (state, frames, events...)
        frames/UP/*.jpg      subsampled UP-camera frames
        frames/FRONT/*.jpg   subsampled FRONT-camera frames

Copy that folder off the phone over USB (Android/data/<rc app>/files/blackbox/)
and run this tool:

    python3 blackbox_builder.py blackbox/teleop_1760000000000
    python3 blackbox_builder.py SESSION_DIR --out my_dataset --copy-frames

Outputs (in --out, default <session>_dataset/):
    state.csv         one row per state/auto/teleop record (pose, RPMs, inputs...)
    events.csv        one row per event (phases, shots, jam coughs, decisions)
    frames.csv        frame file -> wallclock/nano timestamps, per camera
    report.txt        human-readable match summary
    frames/           (with --copy-frames) renamed copies: <camera>_<seq>.jpg

Only uses the Python standard library — no installs needed.
"""

import argparse
import csv
import json
import math
import os
import shutil
import sys

KNOWN_STATE_COLUMNS = [
    "record", "remainingSec", "poseXcm", "poseYcm", "headingRad", "ballsHeld",
    "action", "targetId", "flyL", "flyR", "batteryV", "tiltDeg",
    "fwd", "strafe", "turn", "a", "b", "rb", "lb", "trigger",
    "hiveCm", "hiveBearing", "shootingRpm", "atSpeed", "deadman", "tipping",
    "state",
]


def load_records(session_dir):
    path = os.path.join(session_dir, "state.jsonl")
    if not os.path.exists(path):
        sys.exit("no state.jsonl in {} — is this a Black Box session?".format(session_dir))
    records = []
    with open(path) as f:
        for i, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except ValueError:
                print("! skipping malformed line {}".format(i + 1))
    return records


def split_records(records):
    """Route lines into: state rows, events, frames. Frame lines are the ones
    written by BlackBox.noteFrame(): {"frame": <camera>, "file": ..., ...}."""
    state, events, frames = [], [], []
    for r in records:
        if "event" in r:
            events.append(r)
        elif "file" in r and "frame" in r:
            frames.append(r)
        elif r.get("record") in ("state", "auto", "teleop", "guard", "jam"):
            state.append(r)
    return state, events, frames


def path_length(state_rows):
    """Distance traveled (cm) from pose records, if present."""
    total, last = 0.0, None
    for r in state_rows:
        if "poseXcm" not in r:
            continue
        p = (r["poseXcm"], r["poseYcm"])
        if last is not None:
            total += math.hypot(p[0] - last[0], p[1] - last[1])
        last = p
    return total

def build(args):
    session_dir = args.session
    records = load_records(session_dir)
    state, events, frames = split_records(records)

    out = args.out or (session_dir.rstrip("/").rstrip("\\") + "_dataset")
    os.makedirs(out, exist_ok=True)

    # ---- state.csv ----
    with open(os.path.join(out, "state.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "tn"] + KNOWN_STATE_COLUMNS, extrasaction="ignore")
        w.writeheader()
        for r in state:
            row = {"t": r.get("t"), "tn": r.get("tn")}
            row.update({k: r.get(k, "") for k in KNOWN_STATE_COLUMNS})
            w.writerow(row)

    # ---- events.csv ----
    event_cols = ["t", "event", "source", "action", "targetId", "targetLabel",
                  "value", "costSec", "ballsHeld", "timeRemainingSec", "why",
                  "rpm", "distanceCm", "label", "balls", "name", "currentA",
                  "cough", "alliance", "batteryV"]
    with open(os.path.join(out, "events.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=event_cols, extrasaction="ignore")
        w.writeheader()
        for r in events:
            w.writerow({k: r.get(k, "") for k in event_cols})

    # ---- frames.csv (+ optional copies) ----
    with open(os.path.join(out, "frames.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "camera", "file", "captureNanos"])
        w.writeheader()
        for r in frames:
            w.writerow({"t": r.get("t"), "camera": r.get("frame"),
                        "file": r.get("file"), "captureNanos": r.get("captureNanos")})
    if args.copy_frames:
        for cam in sorted({r.get("frame") for r in frames}):
            src = os.path.join(session_dir, "frames", str(cam))
            if not os.path.isdir(src):
                continue
            dst = os.path.join(out, "frames")
            os.makedirs(dst, exist_ok=True)
            names = sorted(os.listdir(src))
            for i, name in enumerate(names):
                shutil.copy2(os.path.join(src, name),
                             os.path.join(dst, "{}_{:06d}.jpg".format(cam, i)))

    # ---- report.txt ----
    t0 = records[0]["t"] if records else 0
    t1 = records[-1]["t"] if records else 0
    planner = [r for r in events if r.get("event") == "planner_decision"]
    by_action = {}
    for r in planner:
        key = "{}:{}".format(r.get("source", "?"), r.get("action", "?"))
        by_action[key] = by_action.get(key, 0) + 1
    volts = [r["batteryV"] for r in state
             if isinstance(r.get("batteryV"), (int, float)) and r["batteryV"] > 0]
    fly = [r["flyL"] for r in state if isinstance(r.get("flyL"), (int, float))]

    lines = []
    lines.append("BLACK BOX REPORT — {}".format(os.path.basename(session_dir.rstrip("/"))))
    lines.append("=" * 60)
    lines.append("duration:            {:.1f} s (wall clock)".format(max(0.0, (t1 - t0) / 1000.0)))
    lines.append("total log lines:     {}".format(len(records)))
    lines.append("state rows:          {}".format(len(state)))
    lines.append("frames:              {}".format(len(frames)))
    for cam in sorted({str(r.get("frame")) for r in frames}):
        lines.append("    {:>6} frames:  {}".format(cam, sum(1 for r in frames if r.get("frame") == cam)))
    lines.append("events:              {}".format(len(events)))
    lines.append("distance traveled:   {:.0f} cm (from pose records)".format(path_length(state)))
    if volts:
        lines.append("battery:             {:.2f}V min / {:.2f}V max".format(min(volts), max(volts)))
    if fly:
        lines.append("flywheel peak:       {:.0f} RPM".format(max(fly)))
    lines.append("")
    lines.append("planner decisions ({} total):".format(len(planner)))
    for k in sorted(by_action):
        lines.append("    {:<28} {}".format(k, by_action[k]))
    lines.append("")
    lines.append("event timeline:")
    for r in events:
        if r.get("event") == "planner_decision":
            continue
        detail = {k: v for k, v in r.items()
                  if k in ("name", "action", "rpm", "balls", "why", "label")}
        lines.append("    +{:7.1f}s  {} {}".format(
            (r["t"] - t0) / 1000.0, r.get("event", "?"), detail))

    report = "\n".join(lines)
    with open(os.path.join(out, "report.txt"), "w") as f:
        f.write(report + "\n")
    print(report)
    print("\n* dataset written to {}".format(out))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="Black Box session folder (contains state.jsonl)")
    ap.add_argument("--out", default=None, help="output dataset folder (default <session>_dataset)")
    ap.add_argument("--copy-frames", action="store_true",
                    help="also copy + rename frames into the dataset")
    args = ap.parse_args()
    if not os.path.isdir(args.session):
        sys.exit("not a folder: {}".format(args.session))
    build(args)


if __name__ == "__main__":
    main()
