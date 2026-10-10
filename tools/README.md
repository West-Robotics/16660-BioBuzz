# BioBuzz computer tools (Python + OpenCV)

Two desktop tools that support the robot's vision and shooter calibration.
They run on any machine with Python 3 — your Mac is fine.

```bash
cd tools
python3 -m pip install -r requirements.txt
```

## 1. `hsv_calibrator.py` — tune the ball/flower/robot color ranges

A GUI with HSV sliders, live mask preview, automatic bounding boxes, and
mouse-drawn bounding boxes. Run it on a video of the field or a live webcam:

```bash
python3 hsv_calibrator.py --camera 0                 # live webcam
python3 hsv_calibrator.py field_video.mp4            # a recorded video
python3 hsv_calibrator.py video.mp4 --name POLLEN_HSV_RANGE
```

- **Sliders** (H/S/V min & max) live in the "HSV Controls" window. OpenCV's hue
  runs **0–180 — the exact same scale the robot's Kotlin `ColorRange` uses**, so
  values transfer 1:1.
- **Bounding boxes**: the tool draws green/yellow boxes around everything the
  current range matches. To build a range from a sample instead, **drag a box
  around a ball** with the mouse, then press **`f`** — the sliders jump to the
  min/max HSV inside your box. Nudge from there.
- Press **`s`** to print a paste-ready `ColorRange(...)` snippet for `globals.kt`.
- Press **`p`** to save the range as a JSON preset (default `hsv_presets.json`) —
  the flywheel analyzer can reuse it.
- `SPACE` pause, `u` undo box, `q` quit.

## 2. `flywheel_analyzer.py` — automatic shooter calibration from video

Films of the calibration shots are analyzed automatically: it tracks the ball,
finds where each shot lands, converts pixels to cm, and prints the numbers
ready to paste into `globals.kt` (`SHOOTER_CALIBRATION_DISTANCE_CM`).

### Camera setup (do this once)

1. **Fix the camera** (tripod / propped phone). It must not move during filming.
2. Point it **roughly side-on** to the ball's flight path, so the ball's
   *lowest point in the frame* is where it touches the floor.
3. Put **two tape marks on the floor a known distance apart** (e.g. 100 cm),
   in view of the camera, roughly along the flight direction.
4. Tune the ball's HSV first with `hsv_calibrator.py` and save the preset
   (`p`), or plan to pass `--hsv` numbers.

### Robot + filming procedure

1. Run the **"Flywheel Calibration"** OpMode (group *Test* on the Driver
   Station). Pick a RPM step, press **A** to spin up, wait for `AT SPEED`,
   press **X** to feed one ball. Film each shot.
2. Film the shots **in order 6000 → 5000 → 4000 → 3000 → 2000 → 1000** (one
   video is easiest; one video per shot also works — run the analyzer once per
   video with a single `--rpm` value).

### Analyze

```bash
# one video with all six shots (default order 6000..1000):
python3 flywheel_analyzer.py calib.mp4

# ball color from a preset saved by hsv_calibrator.py:
python3 flywheel_analyzer.py calib.mp4 --preset hsv_presets.json --preset-name POLLEN_HSV_RANGE

# or pass the range inline (H_MIN S_MIN V_MIN H_MAX S_MAX V_MAX):
python3 flywheel_analyzer.py calib.mp4 --hsv 14 100 100 35 255 255

# one video per shot:
python3 flywheel_analyzer.py shot_6000.mp4 --rpm 6000
```

The first frame opens a **click-calibration** window:

- Click **1**: the launch point (where the ball exits the flywheel).
- Clicks **2 & 3**: your two tape marks (`--ref-cm 100` by default — change it
  if your marks are a different distance apart).
- `c` confirms, `u` undoes.

It then plays the annotated video while tracking (press `q` to fast-forward
without display) and prints:

- the measured landing distance per RPM,
- a paste-ready `SHOOTER_CALIBRATION_RPM` / `SHOOTER_CALIBRATION_DISTANCE_CM`
  block for `globals.kt` (sorted 6000 → 1000),
- a `*_calibration.json` next to the video with the raw results.

### Finish on the robot

Paste the printed distances into `globals.kt`. That's it — `ShooterModel`
immediately starts interpolating the curve, the shooting window widens to the
calibrated reach, and teleop/autonomous spin the flywheel to the RPM the live
hive distance needs (firing only once **both** wheels are at speed). While
idle the flywheel stays primed at `FLYWHEEL_PRIME_RPM` so spin-up is quick.

### Analyzer assumptions & knobs

- Side-on, static camera (landing = lowest tracked point).
- One ball visible at a time; other same-colored objects near the launcher can
  confuse tracking (move them away while calibrating).
- `--launch-radius` (default 120 px): how near the launch point a new shot
  must appear; raise it if shots aren't detected, lower it if tracks merge.
- `--lost-frames` (default 25): how long the ball can vanish before a track
  ends; raise for low frame-rate videos.
- `--min-area` (default 80 px²): noise floor for the ball blob.
