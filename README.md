# FTC Team 16660 BioBuzz — Complete Operations Manual

## Table of Contents
1. [Getting Started: Environment Setup](#chapter-1-getting-started-environment-setup)
2. [Repository Structure & Code Organization](#chapter-2-repository-structure--code-organization)
3. [Hardware Configuration & Robot Wiring](#chapter-3-hardware-configuration--robot-wiring)
4. [Building & Deploying](#chapter-4-building--deploying)
5. [OpMode Catalog](#chapter-5-opmode-catalog)
6. [Drivetrain Setup & Verification](#chapter-6-drivetrain-setup--verification)
7. [Vision System: Calibration & Tuning](#chapter-7-vision-system-calibration--tuning)
8. [Autonomous Operation](#chapter-8-autonomous-operation)
9. [TeleOp Operation](#chapter-9-teleop-operation)
10. [Shooter Calibration](#chapter-10-shooter-calibration)
11. [Troubleshooting Guide](#chapter-11-troubleshooting-guide)
12. [Advanced: Extending the Codebase](#chapter-12-advanced-extending-the-codebase)
13. [Appendix: All Tunable Constants](#chapter-13-appendix-all-tunable-constants)

---

# Chapter 1: Getting Started — Environment Setup

## 1.1 Prerequisites

### Required Software
| Tool | Version | Purpose |
|------|---------|---------|
| **Android Studio** | Ladybug (2024.2) or later | IDE for Kotlin/Java development |
| **JDK** | 17+ (Gradle 9.1 requirement) | Build runtime |
| **Android SDK** | API 30 (compileSdk), API 28 (targetSdk) | Android build target |
| **Gradle** | 9.1.0 (via wrapper) | Build system |
| **Git** | Latest | Version control |
| **ADB** | Included with Android Studio | Device communication |

### Operating System Support
- **macOS**: Native support (Apple Silicon or Intel)
- **Windows 10/11**: Native support (WSL2 optional but recommended for adb)
- **Linux**: Native support (Ubuntu 22.04+ tested)

## 1.2 Initial Clone & Setup

```bash
# Clone the repository
git clone https://github.com/West-Robotics/16660-BioBuzz.git
cd 16660-BioBuzz

# Verify Gradle wrapper
./gradlew --version

# Create local.properties with your SDK path
# macOS/Linux:
echo "sdk.dir=/Users/$USER/Library/Android/sdk" > local.properties
# Windows (PowerShell):
# "sdk.dir=C:\\Users\\$env:USERNAME\\AppData\\Local\\Android\\Sdk" | Out-File -Encoding utf8 local.properties
```

## 1.3 Android Studio Import

1. Open Android Studio → **File → Open** → select the `16660-BioBuzz` folder
2. Wait for Gradle sync (first sync downloads dependencies ~2-5 min)
3. Verify build: **Build → Make Project** (Ctrl+F9 / Cmd+F9)
4. Run unit tests: `./gradlew :TeamCode:test` (should show 32 tests passing)

## 1.4 Device Setup (Control Hub / Phone)

### Control Hub (REV)
1. Connect via USB or WiFi (ADB over TCP/IP: `adb connect 192.168.x.x:5555`)
2. Enable "USB Debugging" in RC app settings
3. Verify: `adb devices` shows the device

### Robot Configuration
1. On Driver Station: **Program & Manage → Robot Controller → Configure Robot**
2. Create/edit configuration matching **Chapter 3** hardware names
3. Save → Robot Controller restarts automatically

## 1.5 First Build & Deploy

```bash
# Debug APK (faster, for development)
./gradlew :TeamCode:assembleDebug

# Install to connected device
adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk

# Or use Android Studio: Run → Run 'TeamCode' (green play button)
```

**Expected**: App installs, appears on Driver Station under "TeleOp" and "Auto" groups.

---

# Chapter 2: Repository Structure & Code Organization

## 2.1 Top-Level Layout

```
16660-BioBuzz/
├── build.gradle                 # Root: Kotlin 2.2.20, AGP 8.13.2
├── build.common.gradle          # Shared Android config (compileSdk 30, minSdk 24)
├── build.dependencies.gradle    # FTC SDK 12.0.0 dependency versions
├── settings.gradle              # Includes :FtcRobotController, :TeamCode
├── gradle.properties            # AndroidX, JVM args (-Xmx1024M)
├── gradle/wrapper/              # Gradle 9.1.0 wrapper
├── local.properties             # YOUR SDK PATH (gitignored)
├── TeamCode/                    # YOUR ROBOT CODE (this is what you edit)
├── FtcRobotController/          # FTC SDK library (don't edit)
└── tools/                       # Python utilities (HSV calibrator, flywheel analyzer)
```

## 2.2 TeamCode Module Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── globals.kt                   # ALL constants, hardware names, HSV ranges, PID gains
├── motors.kt                    # Drivetrain, Intake, FlyWheel, PickupCurrentSensor, ShotDetector
├── cameras.kt                   # Legacy VisionSystem (dual portal, AprilTag + color blobs)
├── autonomy.kt                  # PhaseClock, ShooterModel, WorldModel (pure Kotlin, JVM-testable)
├── hivescoring.kt               # HiveScoring, BallLocator, TagInfoOverlay
├── BioBuzzAutonomous.kt         # MAIN @Autonomous — score loop
├── BioBuzzTeleOp.kt             # MAIN @TeleOp — driver control + vision assists
├── TestOpModes.kt               # 4 test classes (HSV, localization, camera cal, drivetrain)
├── FlywheelCalibration.kt       # Shooter RPM vs distance calibration OpMode
├── drivetrainspec.kt            # Physical measurements → derived constants (single source of truth)
├── odometry.kt                  # DeadWheelMath (pure math, JVM-testable)
├── debug.kt                     # Overlays: MetadataOverlay, CalibrationProcessor
├── roadrunner/
│   ├── MecanumDrive.kt          # RR 1.0 drive + 2 localizers + PoseCorrector hook
│   ├── Localizer.kt             # Localizer & PoseCorrector interfaces
│   ├── Drawing.kt               # Field overlay helpers
│   └── messages/                # FlightRecorder message types
└── vision/
    ├── VisionConfig.kt          # Camera intrinsics/extrinsics, VisionTuning (@Config)
    ├── FeatureTracking.kt       # Shi-Tomasi + Lucas-Kanade tracker, LandmarkTracker
    ├── VisionCameraProcessor.kt # Per-camera vision pipeline (vision thread)
    ├── SceneEstimation.kt       # RANSAC + VisualMotionEstimator + world map
    └── VisionSubsystem.kt       # NextFTC Mechanism, HybridPoseEstimator, visionGuidance

TeamCode/src/test/               # JVM unit tests (no Android deps)
    ├── DeadWheelMathTest.kt
    ├── PhaseClockTest.kt
    ├── WorldModelTest.kt
    └── DrivetrainSpecTest.kt
```

## 2.3 Key Design Principles

| Principle | Implementation |
|-----------|----------------|
| **Single source of truth** | Physical measurements only in `drivetrainspec.kt`; all derived constants computed |
| **Pure logic separated** | `autonomy.kt`, `odometry.kt`, `drivetrainspec.kt` have zero Android deps → unit testable |
| **Hardware names centralized** | All motor/servo/camera names in `globals.kt` constants |
| **No singletons** | Hardware instances created per OpMode, passed explicitly |
| **NextFTC for actuators** | `Mechanism`, `NextMotor`, `motorEventLoop.poll()` pattern |
| **Road Runner 1.0** | Trajectory following + localization (Kotlin port) |
| **Vision on portal threads** | Control loop consumes immutable snapshots (`AtomicReference`) |
| **All tunables in @Config classes** | Live tuning via FTC Dashboard (no rebuild needed) |

---

# Chapter 3: Hardware Configuration & Robot Wiring

## 3.1 Required Hardware

| Device | Qty | Config Name (must match globals.kt) | Notes |
|--------|-----|--------------------------------------|-------|
| REV 5203 Yellow Jacket 117 RPM | 4 | `frontleft`, `frontright`, `backleft`, `backright` | Mecanum drive |
| REV 5203 6000 RPM (or equivalent) | 2 | `flywheelleft`, `flywheelright` | Opposite directions |
| REV Core Hex / HD Hex | 1 | `intake` | Single motor intake |
| REV Smart Robot Servo | 1 | `loader` | Ball loader/feed |
| REV Control Hub IMU | 1 | `imu` | Built-in or external |
| Dead wheel pods (optional) | 3 | `par0`, `par1`, `perp` | Odometry pods |
| Logitech C270 / REV Cam | 2 | `Webcam High`, `Webcam Ground` | Or `UP_CAM_NAME`, `FRONT_CAM_NAME` |

## 3.2 Motor Directions (CRITICAL — verify on bench)

### Drivetrain (mecanum, +X forward, +Y left, +CCW turn)
| Motor | Config Name | Expected Direction | Verification |
|-------|-------------|-------------------|--------------|
| Front Left | `frontleft` | **FORWARD** | A in DrivetrainSetupOpMode: robot moves forward |
| Front Right | `frontright` | **REVERSE** | |
| Back Left | `backleft` | **FORWARD** | |
| Back Right | `backright` | **REVERSE** | |

**To flip**: Edit `motors.kt` `Drivetrain` constructor → change `direction = DcMotorSimple.Direction.FORWARD/REVERSE`

### Flywheel (opposite spins for straight launch)
| Motor | Config Name | Direction |
|-------|-------------|-----------|
| Left | `flywheelleft` | **FORWARD** |
| Right | `flywheelright` | **REVERSE** |

### Intake
| Motor | Config Name | Direction |
|-------|-------------|-----------|
| Intake | `intake` | **FORWARD** = intake IN |

### Dead Wheels (odometry pods)
| Pod | Config Name | Position (robot frame) | Reversed Flag |
|-----|-------------|------------------------|---------------|
| Parallel 0 | `par0` | (0, +15.2 cm) | `PAR0_REVERSED` |
| Parallel 1 | `par1` | (0, -15.2 cm) | `PAR1_REVERSED` |
| Perpendicular | `perp` | (-15.2, 0 cm) | `PERP_REVERSED` |

**Verify**: DrivetrainSetupOpMode → hold X → hand-push robot → watch encoder signs match movement.

## 3.3 IMU Orientation

In `MecanumDrive.kt` `Params` (also live-tunable via FTC Dashboard @Config):

```kotlin
var logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP   // Logo points UP?
var usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD // USB ports face FORWARD?
```

**Measure physically**: Look at Control Hub mounting. Logo UP = `UP`. USB forward = `FORWARD`. Wrong values = heading drift.

## 3.4 Camera Mounting & Extrinsics

Two cameras in modern vision stack (`vision/VisionConfig.kt`):

| Camera | Config Name | Resolution | Mount | Key Extrinsics |
|--------|-------------|------------|-------|----------------|
| UP | `Webcam High` | 800×448 | Angled up (sees AprilTags) | `translationTrust=0`, `headingTrust=1` |
| FRONT | `Webcam Ground` | 640×480 | Forward (sees floor) | `translationTrust=1`, `headingTrust=0.7` |

**Extrinsics to measure** (tape measure + protractor):
- `position`: (x, y, z) cm from robot center
- `yawDeg`, `pitchDeg`, `rollDeg`: Camera rotation relative to robot
- `imageToRobotYawDeg`: Yaw from image +X to robot +X (MEASURE: Test 3 in VisualLocalizationTest)
- `sceneDistanceCm`: Effective depth for monocular translation
- `headingSign`: +1 or -1 (flip if heading correction goes wrong way)

---

# Chapter 4: Building & Deploying

## 4.1 Gradle Commands

```bash
# Clean build
./gradlew clean

# Build debug APK (development)
./gradlew :TeamCode:assembleDebug

# Build release APK (competition)
./gradlew :TeamCode:assembleRelease

# Run JVM unit tests (no device needed)
./gradlew :TeamCode:test

# Run connected Android tests (requires device)
./gradlew :TeamCode:connectedAndroidTest

# Stop all Gradle daemons (fixes journal lock issues)
./gradlew --stop
```

## 4.2 Build Variants

| Variant | Use Case | Speed |
|---------|----------|-------|
| `debug` | Development, testing | Fast |
| `release` | Competition, final | Slower (minify, shrink) |

## 4.3 Common Build Issues

| Error | Fix |
|-------|-----|
| `SDK location not found` | Create `local.properties` with `sdk.dir` |
| `Gradle daemon journal locked` | `./gradlew --stop` then retry |
| `OutOfMemoryError` | Increase `org.gradle.jvmargs=-Xmx2048M` in `gradle.properties` |
| `Duplicate class` | Clean build: `./gradlew clean :TeamCode:assembleDebug` |
| `Annotation processor not found` | Sync project (File → Sync Project with Gradle Files) |

## 4.4 Deploying to Robot

### Via Android Studio (Recommended)
1. Connect Control Hub via USB
2. Select `TeamCode` run configuration (top toolbar)
3. Click **Run** (green play) → selects device → installs & launches

### Via ADB (Command Line)
```bash
# Install
adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk

# Uninstall old version first (if needed)
adb uninstall org.firstinspires.ftc.teamcode

# Launch app
adb shell am start -n org.firstinspires.ftc.teamcode/org.firstinspires.ftc.robotcontroller.FtcRobotControllerActivity
```

### Via FTC Dashboard (Wireless)
1. Connect to RC WiFi Direct or same network
2. Open `http://<rc-ip>:8080` in browser
3. Use "Files" tab to upload APK, or "OpModes" to select active OpMode

---

# Chapter 5: OpMode Catalog

## 5.1 Competition OpModes

### BioBuzz Autonomous (`BioBuzzAutonomous.kt`)
- **Annotation**: `@Autonomous(name="BioBuzz Autonomous", group="Auto")`
- **Purpose**: Main 30-second autonomous
- **INIT controls**: D-pad LEFT=Red alliance, RIGHT=Blue alliance
- **Phases**: COLLECT → (FLOWER) → POLISH → ALIGN → FIRE → PARK
- **Requires**: Full hardware (drive, IMU, both cameras, flywheel, intake, loader)

### BioBuzz TeleOp (`BioBuzzTeleOp.kt`)
- **Annotation**: `@TeleOp(name="BioBuzz TeleOp", group="TeleOp")`
- **Purpose**: Driver-controlled with vision assists
- **Gamepad 1 Controls**:
  - Left stick: Drive (forward/strafe)
  - Right stick X: Turn
  - **L-Stick Button**: Toggle field-centric
  - **R-Stick Button**: Toggle pose hold
  - **LB (hold)**: Ball capture assist (strafe to nearest ball)
  - **RB (hold)**: Shoot align assist (auto-align to hive + auto-feed)
  - **RT > 0.3**: Manual fire at calibrated RPM
  - **A**: Intake IN
  - **B**: Intake REVERSE (eject)
  - **D-pad**: Alliance selection during INIT

## 5.2 Test & Calibration OpModes

| OpMode | Group | Purpose | Key Controls |
|--------|-------|---------|--------------|
| **Vision Test** | Test | Live HSV tuning for all game pieces | Bumpers=cycle target, D-pad=select param, D-pad L/R=tweak, X=10×, A=overlay |
| **Visual Localization Test** | Test | 6-step vision validation (no drive) | Watch telemetry; all 6 tests must pass |
| **Camera Calibration (C270)** | Test | Measure focal length for both cameras | D-pad=distance/tag width, A=average, writes VisionConfig.kt |
| **Drivetrain Setup & Verify** | Test | Bring-up bench: verify all constants | A=1s spin, B=50cm crawl, X(hold)=live encoders |
| **Flywheel Calibration** | Test | RPM vs distance curve | A=spin toggle, X=feed ball, use with `tools/flywheel_analyzer.py` |
| **Drive Only** | Test | Hub + motors only (no vision) | Basic drive test |
| **Neural Net Test** | Test | (Legacy) Model loading test | N/A — classical CV used |
| **Experiment** | Test | Configurable bench sequences | Edit `globals.kt` `EXPERIMENT_*` constants |

## 5.3 OpMode Registration

All OpModes use standard FTC annotations (`@TeleOp`, `@Autonomous`). **No manual register file needed** — SDK 12+ auto-discovers via classpath scanning.

**To add new OpMode**:
1. Create class extending `LinearOpMode`
2. Add `@TeleOp(name="My Name", group="MyGroup")` or `@Autonomous(...)`
3. Build → appears on Driver Station automatically

---

# Chapter 6: Drivetrain Setup & Verification

## 6.1 Physical Measurements (Edit `drivetrainspec.kt` ONLY)

```kotlin
// MEASURE THESE PHYSICALLY WITH TAPE MEASURE:
const val DRIVE_WHEEL_DIAMETER_CM = 5.0          // Wheel OD
const val DRIVE_MOTOR_CPR = 28                   // Motor encoder counts per rev (before gearbox)
const val DRIVE_GEAR_RATIO = 51.46               // Gearbox ratio (Yellow Jacket 117 RPM = 51.46:1)
const val TRACK_WIDTH_CM = 87.0                  // Center-to-center left/right wheels
const val WHEELBASE_CM = 87.0                    // Center-to-center front/back wheels
const val MECANUM_LATERAL_SLIP = 1.0             // 1.0 = ideal, >1 = more slip

// Dead wheels (if installed):
const val DEAD_WHEEL_DIAMETER_CM = 3.5
const val DEAD_WHEEL_TICKS_PER_REV = 2000        // Encoder CPR on pod
const val DEAD_WHEEL_PAR0_POS_CM = Pair(0.0, 15.2)
const val DEAD_WHEEL_PAR1_POS_CM = Pair(0.0, -15.2)
const val DEAD_WHEEL_PERP_POS_CM = Pair(-15.2, 0.0)
const val PAR0_REVERSED = false
const val PAR1_REVERSED = false
const val PERP_REVERSED = false
```

**NEVER edit derived constants** (`driveCmPerTick`, `rrInPerTick`, `deadWheelInPerTick`, etc.) — they recompute automatically.

## 6.2 Verification Procedure (DrivetrainSetupOpMode)

### Step 1: Spin Test (A button)
1. Robot on blocks (wheels free)
2. Press **A** → each motor spins 1s at 0.3 power
3. **Verify**: All wheels spin FORWARD (robot would move forward)
4. If any wheel spins backward → flip its `direction` in `motors.kt`

### Step 2: 50 cm Crawl (B button)
1. Robot on floor, tape measure alongside
2. Press **B** → robot drives forward 50 cm at 0.15 speed
3. **Measure actual distance** → should be 50 cm ± 1 cm
4. If off → check `DRIVE_WHEEL_DIAMETER_CM`, `DRIVE_GEAR_RATIO`, `TRACK_WIDTH_CM`

### Step 3: Live Encoders (Hold X)
1. Hold **X** → telemetry shows all 4 drive encoders + dead wheels (if present)
2. Hand-push robot forward → all 4 drive encoders should **increase**
3. Hand-push left → left wheels increase, right decrease
4. Hand-push rotate CCW → LF/LB increase, RF/RB decrease
5. Dead wheels: verify signs match `drivetrainspec.kt` positions

### Step 4: IMU Heading
1. Telemetry shows `imuYawDeg` (0° = forward, +CCW)
2. Rotate robot CCW by 90° → yaw should read ~+90°
3. If sign wrong → check `logoFacingDirection` / `usbFacingDirection` in `MecanumDrive.Params`

## 6.3 Road Runner Params Tuning (FTC Dashboard)

Open `http://<rc-ip>:8080` → **Config** → `MecanumDrive$Params`

### Feedforward First (do this before PID gains)
| Param | Typical Range | How to Tune |
|-------|---------------|-------------|
| `kS` | 0.1–0.5 | Static friction: slowly increase until robot *just* starts moving at min power |
| `kV` | 0.05–0.15 | Velocity: drive at constant velocity, adjust until actual ≈ target |
| `kA` | 0.001–0.01 | Acceleration: rapid accel/decel, adjust to minimize following error |

### PID Gains (after FF tuned)
| Param | Typical Range | Effect |
|-------|---------------|--------|
| `axialGain` | 2–8 | Forward/back position P |
| `lateralGain` | 2–8 | Strafe position P |
| `headingGain` | 3–10 | Turn position P |
| `axialVelGain` | 0.5–2 | Forward/back velocity D |
| `lateralVelGain` | 0.5–2 | Strafe velocity D |
| `headingVelGain` | 1–3 | Turn velocity D |

**Tune order**: FF → axial/lateral/heading gains → vel gains.

---

# Chapter 7: Vision System — Calibration & Tuning

## 7.1 Two Vision Stacks (Choose One)

### Legacy (`cameras.kt` → `VisionSystem`)
- Dual VisionPortal: HIGH (AprilTag) + GROUND (color blobs)
- Simpler, proven, used by current autonomous
- **HSV tuning**: `VisionTestOpMode`

### Modern (`vision/` package — Hybrid Localization)
- Feature tracking (Shi-Tomasi + LK) + RANSAC + landmark mapping
- Fuses with RR pose for drift correction
- **Not yet enabled by default** in autonomous (see `FUSION_ENABLED_DEFAULT=false`)

## 7.2 HSV Range Tuning (Legacy + Modern Color Blobs)

Run **VisionTestOpMode**:

```
Controls:
  Bumpers (LB/RB)     → Cycle through targets: Pollen, Nectar Blue, Nectar Red, Nectar Red Wrap, Flower, Robot
  D-pad Up/Down       → Select H/S/V Min/Max
  D-pad Left/Right    → Adjust value (-1)
  X + D-pad Left/Right→ Adjust value (-10 / +10)
  A                   → Toggle calibration overlay (shows detected blobs)
```

**Workflow**:
1. Place game pieces in various lighting
2. Adjust until blobs tightly hug pieces, no noise
3. Press **A** to verify overlay matches
4. Copy values from telemetry → paste into `globals.kt` HSV constants

### Current Defaults (globals.kt)
```kotlin
// OpenCV HSV: H=0-180, S=0-255, V=0-255
val POLLEN_HSV_RANGE = intArrayOf(14, 35, 100, 255, 100, 255)
val NECTAR_BLUE_HSV_RANGE = intArrayOf(100, 140, 100, 255, 100, 255)
val NECTAR_RED_HSV_RANGE = intArrayOf(0, 12, 100, 255, 100, 255)
val NECTAR_RED_WRAP_HSV_RANGE = intArrayOf(168, 180, 100, 255, 100, 255)
val FLOWER_GREEN_HSV_RANGE = intArrayOf(45, 80, 50, 255, 50, 255)
val ROBOT_DARK_HSV_RANGE = intArrayOf(0, 180, 0, 120, 0, 90)
```

## 7.3 Camera Calibration (CameraCalibrationOpMode)

**Required for**: Modern vision stack (extrinsics), legacy AprilTag distance accuracy

### Procedure:
1. Print AprilTag (Tag ID 0, 150mm/6in wide) on flat matte surface
2. Mount camera at competition height/angle
3. Measure exact distance from camera lens to tag (tape measure)
4. Run **Camera Calibration (C270)** OpMode
5. D-pad Up/Down = adjust known distance
6. D-pad Left/Right = adjust tag width
7. Hold **A** → averages 30 frames → displays focal length
8. **Write values to `VisionConfig.kt`**:
   ```kotlin
   // UP camera (800x448)
   val UP_CAMERA = CameraIntrinsics(
       focalLengthPx = YOUR_MEASURED_VALUE,
       centerX = 400, centerY = 224,  // half resolution
       distortion = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0)
   )
   // FRONT camera (640x480)
   val FRONT_CAMERA = CameraIntrinsics(
       focalLengthPx = YOUR_MEASURED_VALUE,
       centerX = 320, centerY = 240,
       distortion = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0)
   )
   ```

## 7.4 Modern Vision Extrinsics (VisionConfig.kt)

**Measure physically** (critical for fusion accuracy):

```kotlin
// UP Camera (angled up, sees tags)
val UP_CAMERA = CameraExtrinsics(
    position = Pair(0.0, 0.0),  // robot center (cm)
    yawDeg = 0.0,               // camera forward = robot forward
    pitchDeg = -30.0,           // angled UP 30°
    rollDeg = 0.0,
    imageToRobotYawDeg = -90.0, // MEASURE: image +X → robot +X
    sceneDistanceCm = 120.0,    // effective tag distance
    headingSign = 1.0,          // +1 or -1 (flip if heading correction wrong)
    translationTrust = 0.0,     // UP cam: NO translation (only heading)
    headingTrust = 1.0          // UP cam: FULL heading trust
)

// FRONT Camera (forward, sees floor)
val FRONT_CAMERA = CameraExtrinsics(
    position = Pair(0.0, 15.0), // 15cm forward of center
    yawDeg = 0.0,
    pitchDeg = -15.0,           // slightly down
    rollDeg = 0.0,
    imageToRobotYawDeg = 0.0,   // image +X = robot +X
    sceneDistanceCm = 60.0,     // floor features ~60cm
    headingSign = 1.0,
    translationTrust = 1.0,     // FRONT cam: FULL translation
    headingTrust = 0.7          // partial heading (floor features noisy)
)
```

## 7.5 VisionTuning (@Config — Live via Dashboard)

Open Dashboard → Config → `VisionTuning`

| Parameter | Default | When to Adjust |
|-----------|---------|----------------|
| `MAX_FEATURES` | 60 | More features = better tracking, slower |
| `QUALITY_LEVEL` | 0.05 | Lower = more features, more noise |
| `MIN_DISTANCE_PX` | 15 | Prevent feature clustering |
| `LK_WIN_SIZE` | 21 | Larger = more robust, slower |
| `LK_MAX_LEVEL` | 3 | Pyramid levels |
| `LK_MAX_ERROR_PX` | 4.0 | Track rejection threshold |
| `MIN_TRACKS` | 25 | Reacquire if below this |
| `REACQUIRE_INTERVAL_FRAMES` | 15 | Periodic full re-detection |
| `RANSAC_ITERATIONS` | 40 | More = robust, slower |
| `RANSAC_INLIER_THRESHOLD_PX` | 3.0 | Pixel tolerance for inlier |
| `RANSAC_MIN_INLIERS` | 12 | Minimum for valid transform |
| `FUSION_MIN_CONFIDENCE` | 0.35 | Below = ignore correction |
| `FUSION_MAX_GAIN` | 0.20 | Max correction per frame |
| `FUSION_MAX_STEP_IN` | 0.30 | Max position jump (inches) |
| `FUSION_MAX_STEP_DEG` | 2.0 | Max heading jump (degrees) |
| `FUSION_RESEED_MS` | 2500 | Re-seed visual pose after loss |
| `GUIDANCE_K_FORWARD` | 0.10 | Vision guidance P gains |
| `GUIDANCE_K_STRAFE` | 0.10 | |
| `GUIDANCE_K_TURN` | 0.02 | |
| `GUIDANCE_POS_TOL_IN` | 0.75 | Settle tolerance |
| `GUIDANCE_HEADING_TOL_DEG` | 3.0 | |
| `GUIDANCE_SETTLE_FRAMES` | 5 | Frames within tolerance to finish |
| `GUIDANCE_TIMEOUT_MS` | 4000 | Max guidance time |
| `GUIDANCE_CONFIDENCE_FLOOR` | 0.45 | Min confidence to engage |

## 7.6 Validating Vision Before Autonomous

Run **Visual Localization Test** OpMode — 6 automated tests:

1. **Stationary** — robot still, features stable
2. **Rotation** — rotate in place, heading tracks
3. **Translation** — drive straight, position tracks
4. **Obstruction** — block camera briefly, recovers
5. **One Camera Blocked** — single camera loss handled
6. **Both Blocked** — total loss, dead-reckons

**All 6 must pass** before setting `FUSION_ENABLED_DEFAULT=true` or enabling `visionCorrectionEnabled` in autonomous.

---

# Chapter 8: Autonomous Operation

## 8.1 Overview

**BioBuzzAutonomous.kt** — Time-budgeted score loop using `PhaseClock` (30s period).

### Alliance Selection (During INIT)
- **D-pad LEFT** → Red alliance
- **D-pad RIGHT** → Blue alliance
- Telemetry confirms: "ALLIANCE: RED/BLUE"
- **Critical**: Wrong alliance = robot scores for opponent!

## 8.2 Phase Clock (autonomy.kt)

```kotlin
class PhaseClock(periodSec = 30.0) {
    fun elapsedSec(): Double
    fun remainingSec(): Double
    fun fits(sec: Double): Boolean
    fun deadlineNanos(phaseTimeoutMs: Double, reserveSec: Double = 0.0): Long
}
```

**Reserves** (globals.kt):
- `AUTONOMY_PARK_RESERVE_SEC = 3.0` — always save for park
- `AUTONOMY_FIRE_RESERVE_SEC = 6.0` — always save for firing
- `MIN_CYCLE_REMAINING_SEC = 14.0` — min time for another full cycle

**Logic**: Phase deadlines are clamped to `remainingSec - reserveSec`. If a phase would exceed budget, it's skipped.

## 8.3 Phase Breakdown

### Phase 1: COLLECT (`collectBalls`)
- **Timeout**: `AUTONOMOUS_COLLECT_TIMEOUT_MS = 9000`
- **Behavior**:
  1. Intake ON (full speed)
  2. Commits to one tracked ball (hysteresis `TARGET_SWITCH_MARGIN = 1.5`)
  3. Dodges robots in corridor (`AVOID_ROBOT_BEARING_DEG = 15°`, latched `DODGE_LATCH_MS = 600`)
  4. Pursues with turn-toward (`CAPTURE_TURN_GAIN = 0.008`)
  4. Pickup confirmed by: current bump (`PICKUP_CURRENT_BUMP_A = 1.5A`) OR proximity settle (`PICKUP_CONFIRM_CM = 25`, `PICKUP_SETTLE_MS = 400`)
  5. If no target → sweep search (`SEARCH_SWEEP_TURN_POWER = 0.12`)

### Phase 1b: FLOWER (`useFlowerIfVisible`)
- **Timeout**: `FLOWER_PULL_TIMEOUT_MS = 6000`
- **Trigger**: Live or remembered flower visible
- **Action**: Drive to `FLOWER_APPROACH_CM = 25`, stop, run `flowerPullRoutine` (customizable in globals.kt), settle intake

### Phase 2: POLISH (`runGuidance`)
- **Cycle 1 only**: Runs vision guidance command once to clean pose drift
- Uses `VisionSubsystem.visionGuidance()` with target from hive

### Phase 3: ALIGN (`alignToHive`)
- **Timeout**: `ALIGN_TIMEOUT_MS = 8000`
- **Live tag**: Uses `HiveScoring.rangeCorrection(target)` → forward/turn correction
- **Tag lost**: Dead-reckons to remembered hive (`world.hive()`)
- **Nothing remembered**: Sweep search

### Phase 4: FIRE (`fireBalls`)
- **Per-ball sequence**:
  1. Spin to `shootingRpmFor(target)` (calibrated curve or 6000 fallback)
  2. Wait for `flywheel.isAtSpeed(targetRpm, toleranceRpm=150)`
  3. Feed pulse: `loader.stroke()` → `FLYWHEEL_FEED_TIME_MS = 1200ms`
  4. `ShotDetector.arm()` → watch for RPM dip > `SHOT_DIP_THRESHOLD_RPM = 250` within 900ms
  5. Only confirmed shots counted; recover spin between shots

### Phase 5: PARK (`runAction`)
- Backs off `PARK_BACKOFF_IN = 12in` along robot's **current heading** (not world X)
- Uses RR trajectory action

## 8.4 WorldModel (Persistent Memory)

```kotlin
// In autonomy.kt
data class TrackedObject(
    val id: Int, val label: String,
    val fieldXcm: Double, val fieldYcm: Double,
    val confidence: Double, val lastSeenNanos: Long,
    val velX: Double, val velY: Double,
    val isBall: Boolean, val lastAreaPx: Int
)
```

- **Fuses sightings** into field coordinates when pose available
- **Remembers hive** from AprilTag sightings
- **Confidence decay**: `exp(-dt / WM_CONFIDENCE_TAU_SEC)` (τ=10s)
- **Track association**: `WM_TRACK_MATCH_CM = 40` max distance
- **Max age**: balls 45s, robots 6s
- **Chase utility**: `bestBallToChase()` — capacity-aware, filters by age ≤ 3s (`DEAD_RECKON_MAX_AGE_SEC`) and area ≥ 350px (`CHASE_MIN_CONTOUR_AREA_PX`)

## 8.5 Autonomous Tuning Constants (globals.kt)

| Constant | Default | Description |
|----------|---------|-------------|
| `AUTONOMOUS_COLLECT_TIMEOUT_MS` | 9000 | Max time per collect attempt |
| `FLOWER_PULL_TIMEOUT_MS` | 6000 | Max time for flower routine |
| `ALIGN_TIMEOUT_MS` | 8000 | Max time to align to hive |
| `CAPTURE_TURN_GAIN` | 0.008 | Turn toward ball gain |
| `CAPTURE_TURN_MAX_POWER` | 0.20 | Max turn power during capture |
| `TARGET_SWITCH_MARGIN` | 1.5 | Hysteresis for switching targets |
| `AVOID_ROBOT_BEARING_DEG` | 15.0 | Dodge if robot within this bearing |
| `DODGE_LATCH_MS` | 600 | Dodge command latch duration |
| `PICKUP_CURRENT_BUMP_A` | 1.5 | Current threshold for pickup detect |
| `PICKUP_CONFIRM_CM` | 25.0 | Proximity confirm distance |
| `PICKUP_SETTLE_MS` | 400 | Settle time after proximity |
| `SEARCH_SWEEP_TURN_POWER` | 0.12 | Sweep search turn power |
| `FLYWHEEL_FEED_TIME_MS` | 1200 | Loader stroke duration |
| `SHOT_DIP_THRESHOLD_RPM` | 250 | RPM dip to confirm shot |
| `SHOT_DETECT_WINDOW_MS` | 900 | Time window to detect dip |
| `PARK_BACKOFF_IN` | 12.0 | Park backoff distance (inches) |

---

# Chapter 9: TeleOp Operation

## 9.1 Driver Controls (BioBuzzTeleOp.kt)

### Drive (Gamepad 1)
| Input | Action | Notes |
|-------|--------|-------|
| Left Stick Y | Forward/Back | Inverted (push forward = positive) |
| Left Stick X | Strafe Left/Right | +X = RIGHT |
| Right Stick X | Turn | +X = CCW |
| **L-Stick Button** | Toggle Field-Centric | Telemetry shows "FIELD" / "ROBOT" |
| **R-Stick Button** | Toggle Pose Hold | Holds position when sticks released |

### Vision Assists
| Input | Assist | Behavior |
|-------|--------|----------|
| **LB (hold)** | Ball Capture | Strafe toward nearest ball + approach assist |
| **RB (hold)** | Shoot Align | Auto-align to hive (range + bearing) + auto-feed when wheels at speed |

### Flywheel & Intake
| Input | Action |
|-------|--------|
| **RT > 0.3** | Manual fire at `shootingRpmFor(target)` |
| **RB assist + ALIGNED** | Auto-fire (same as RT) |
| **Idle + PRIME_ENABLED** | Prime at `FLYWHEEL_PRIME_RPM = 2000` |
| **A** | Intake IN (speed 1.0) |
| **B** | Intake REVERSE (eject, speed -1.0) |
| **Neither** | Intake stop |

### Pose Hold (R-Stick Button Toggle)
When enabled and sticks centered:
- Position P: `TELEOP_HOLD_POSITION_KP = 0.06` per inch
- Heading P: `TELEOP_HOLD_HEADING_KP = 0.02` per degree
- Max power: `TELEOP_HOLD_MAX_POWER = 0.25`

## 9.2 TeleOp Tuning (globals.kt)

| Constant | Default | Adjust If... |
|----------|---------|--------------|
| `TELEOP_FIELD_CENTRIC_DEFAULT` | false | Want field-centric by default |
| `TELEOP_POSE_HOLD_DEFAULT` | true | Want pose hold by default |
| `TELEOP_INPUT_DEADZONE` | 0.08 | Sticks drift at center |
| `TELEOP_HOLD_POSITION_KP` | 0.06 | Pose hold too weak/strong |
| `TELEOP_HOLD_HEADING_KP` | 0.02 | Heading hold too weak/strong |
| `TELEOP_HOLD_MAX_POWER` | 0.25 | Pose hold fights driver |

## 9.3 Vision Assist Tuning (HiveScoringConfig in hivescoring.kt)

| Constant | Default | Effect |
|----------|---------|--------|
| `RANGE_GAIN` | 0.004 | Forward assist strength |
| `BEARING_GAIN` | 0.02 | Turn assist strength |
| `ASSIST_MAX_POWER` | 0.30 | Max assist power (cap) |
| `CAPTURE_RANGE_GAIN` | 0.02 | Ball capture forward gain |
| `CAPTURE_BEARING_GAIN` | 0.02 | Ball capture strafe gain |

---

# Chapter 10: Shooter Calibration

## 10.1 Calibration Procedure

### Equipment Needed
- Tape measure (30 ft)
- Charged battery
- Video recording device (phone)
- `tools/flywheel_analyzer.py` (Python 3 + OpenCV + numpy)

### Steps
1. **Run FlywheelCalibration OpMode**
2. **A button**: Toggle flywheel spin at current RPM step
3. **X button**: Feed one ball (loader strokes)
4. **Record video** of each shot (side view, see ball arc)
5. **D-pad Up/Down**: Change RPM step (6 steps: 6000, 5500, 5000, 4500, 4000, 3500)
6. **Repeat** at multiple distances (measure from launch point to hive)

### Analyze Video
```bash
cd tools
pip install -r requirements.txt
python flywheel_analyzer.py --video shot_6000rpm_200cm.mp4 --rpm 6000 --distance-cm 200
```
Output: measured distance for that RPM.

### Update globals.kt
```kotlin
// Shooter calibration data (RPM → distance cm)
// Fill from flywheel_analyzer.py results
const val SHOOTER_CALIBRATION_RPM = doubleArrayOf(3500.0, 4000.0, 4500.0, 5000.0, 5500.0, 6000.0)
const val SHOOTER_CALIBRATION_DISTANCE_CM = doubleArrayOf(
    120.0,  // measured at 3500 RPM
    150.0,  // measured at 4000 RPM
    185.0,  // measured at 4500 RPM
    225.0,  // measured at 5000 RPM
    270.0,  // measured at 5500 RPM
    320.0   // measured at 6000 RPM
)
```

**Requirements**: ≥2 real data points for `ShooterModel.isCalibrated()` → true. Outside range clamps to nearest.

## 10.2 ShooterModel Usage (autonomy.kt)

```kotlin
val model = ShooterModel()
val rpm = model.rpmForDistance(targetDistanceCm)  // Interpolates curve
val maxRange = model.effectiveMaxRangeCm()        // Max calibrated or fallback
```

Autonomous uses this automatically via `HiveScoring.shootingRpmFor(target)`.

---

# Chapter 11: Troubleshooting Guide

## 11.1 Quick Diagnostic Checklist

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Robot doesn't move in auto | RR Params all zero | Tune MecanumDrive.Params (Chapter 6.3) |
| Robot drives sideways | Motor directions wrong | DrivetrainSetupOpMode spin test (Chapter 6.2) |
| Heading drifts / wrong | IMU orientation wrong | Measure logo/USB facing (Chapter 3.3) |
| Autonomous targets wrong hive | Alliance selection wrong | D-pad LEFT=Red, RIGHT=Blue during INIT |
| Vision assists don't work | HSV ranges off | VisionTestOpMode tuning (Chapter 7.2) |
| AprilTag distance wrong | Focal length wrong | CameraCalibrationOpMode (Chapter 7.3) |
| Flywheel never "at speed" | `isAtSpeed` tolerance too tight | Increase `toleranceRpm` or check `kV/kP` |
| Shots fall short | Shooter uncalibrated | Run FlywheelCalibration (Chapter 10) |
| Intake doesn't pick up | Current sensor threshold wrong | Measure empty intake current, adjust `PICKUP_CURRENT_BUMP_A` |
| Dead wheels not detected | Motor names mismatch | Check config names = `par0`/`par1`/`perp` |
| OpMode not on Driver Station | Missing annotation | Add `@TeleOp`/`@Autonomous` to class |

## 11.2 Common Error Messages

| Error | Location | Fix |
|-------|----------|-----|
| `CANCELLED_FOR_SAFETY` | Actuator write in `stop()` | Move actuator writes to `finally` block or before `return` |
| `HardwareMap.get() throws` | Missing device in config | Add device to Robot Configuration with exact name from globals.kt |
| `IndexOutOfBounds` in vision | AprilTag ID out of range | Check `AprilTagGameDatabase.getCurrentGameTagLibrary()` |
| `NoSuchMethodError: NextMotor` | NextFTC version mismatch | Sync `build.dependencies.gradle` NextFTC version |
| `FlightRecorder buffer full` | Too many log messages | Reduce logging frequency or increase buffer |

## 11.3 Debugging Tools

### FTC Dashboard (http://<rc-ip>:8080)
- **Config**: Live tune all @Config classes (RR Params, VisionTuning, etc.)
- **Telemetry**: Real-time plots, field overlay
- **Camera Stream**: Live camera preview with overlays
- **Files**: Browse/download logs, screenshots

### ADB Logcat
```bash
# Filter to teamcode tag
adb logcat -s "TeamCode:*" "FTC:*" "RR:*" "*:E"

# Save to file
adb logcat -d > logcat.txt
```

### On-Device Logs
- RC App → **Manage → Logs** → View/Download
- Exception popups show 15-line stacktrace (tap OK to continue)

### Telemetry Keys to Watch
| Key | Expected | If Wrong... |
|-----|----------|-------------|
| `drive.estimatedPose` | Updates smoothly | Localizer broken |
| `vision.up.confidence` / `vision.front.confidence` | >0.35 when tracking | Extrinsics/camera issues |
| `world.balls.count` | >0 when balls visible | HSV ranges / lighting |
| `flywheel.measuredAverageRpm` | ≈ target ±150 | FF gains / mechanical |
| `intake.currentAmps` | ~0.5–1.5A running, spikes on ball | Current sensor / wiring |

## 11.4 Field-Test Checklist (Before Every Match)

- [ ] DrivetrainSetupOpMode: Spin test (A) → all wheels forward
- [ ] DrivetrainSetupOpMode: 50cm crawl (B) → measures 50±1 cm
- [ ] DrivetrainSetupOpMode: Live encoders (X) → signs correct
- [ ] IMU heading: CCW rotation = positive yaw
- [ ] FlywheelCalibration: RPM steps smooth, no vibration
- [ ] VisionTestOpMode: HSV ranges tight on all game pieces
- [ ] VisualLocalizationTest: All 6 tests pass (if using modern vision)
- [ ] Autonomous: Alliance selection correct (D-pad)
- [ ] Battery: >12.5V under load
- [ ] All cables secured, no loose connections

---

# Chapter 12: Advanced — Extending the Codebase

## 12.1 Adding a New OpMode

```kotlin
// MyNewOpMode.kt
package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "My New OpMode", group = "Custom")
class MyNewOpMode : LinearOpMode() {
    override fun runOpMode() {
        // 1. Initialize hardware
        val drivetrain = Drivetrain(hardwareMap)
        val intake = Intake(hardwareMap)
        val flywheel = FlyWheel(hardwareMap)
        val loader = Loader(hardwareMap)
        
        // 2. Wait for start
        waitForStart()
        
        // 3. Main loop
        while (opModeIsActive()) {
            // Call motorEventLoop.poll() EVERY LOOP for NextMotor updates
            NextMotor.motorEventLoop.poll()
            
            // Your logic here
            telemetry.update()
        }
        
        // 4. Cleanup (stop motors, close vision)
        drivetrain.flushMotors()
        intake.stop()
        flywheel.stop()
    }
}
```

**Critical**: Call `NextMotor.motorEventLoop.poll()` every loop iteration — without it, velocity/position control won't update!

## 12.2 Adding a New Hardware Device

1. Add config name to `globals.kt`:
   ```kotlin
   const val MY_NEW_MOTOR = "my_motor"
   ```

2. Create mechanism class in `motors.kt` (follow existing patterns):
   ```kotlin
   class MyMechanism(hardwareMap: HardwareMap) : Mechanism {
       val motor = NextMotor(hardwareMap, MY_NEW_MOTOR, direction = FORWARD)
           .apply { setPositionConstants(kV = 1.0/maxRpm, kP = 1.0/maxRpm, kS = 0.03) }
   }
   ```

3. Add to Robot Configuration on Driver Station with exact name `"my_motor"`

## 12.3 Adding a Vision Processor

### Legacy (VisionPortal)
```kotlin
val processor = AprilTagProcessor.Builder()
    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
    .build()

val portal = VisionPortal.Builder()
    .addProcessor(processor)
    .setCamera(hardwareMap.get(WebcamName::class.java, "my_cam"))
    .build()
```

### Modern (VisionSubsystem)
Add camera to `VisionConfig.kt` with intrinsics/extrinsics, then:
```kotlin
val vision = VisionSubsystem(hardwareMap)
vision.attachTo(drive, enableCorrections = true)
```

## 12.4 Modifying Autonomous Logic

### Add a New Phase
1. Add phase constant to `globals.kt`
2. Add phase method to `BioBuzzAutonomous.kt`
3. Insert into `runOpMode()` phase sequence
4. Update `PhaseClock` reserves if needed

### Adjust Collect Behavior
Edit constants in `globals.kt` — no code changes needed for:
- Timeouts, gains, thresholds, priorities

### Change Planner Logic
Edit `StrategicPlanner.decide()` in `autonomy.kt` (pure Kotlin — unit testable!)

## 12.5 Unit Testing New Logic

```bash
# Add test to TeamCode/src/test/
# Run all tests
./gradlew :TeamCode:test

# Run specific test
./gradlew :TeamCode:test --tests "org.firstinspires.ftc.teamcode.DeadWheelMathTest"
```

**Rules**: Test classes must be pure Kotlin (no Android deps). Use `autonomy.kt`, `odometry.kt`, `drivetrainspec.kt` as models.

---

# Chapter 13: Appendix — All Tunable Constants

## 13.1 globals.kt — Complete Reference

### Hardware Names
```kotlin
DrivetrainFrontLeft = "frontleft"
DrivetrainFrontRight = "frontright"
DrivetrainBackLeft = "backleft"
DrivetrainBackRight = "backright"
FlywheelLeftMotor = "flywheelleft"
FlywheelRightMotor = "flywheelright"
IntakeMotor = "intake"
LoaderServoName = "loader"
DEAD_WHEEL_PAR0_NAME = "par0"
DEAD_WHEEL_PAR1_NAME = "par1"
DEAD_WHEEL_PERP_NAME = "perp"
RR_IMU_NAME = "imu"
GROUND_CAM_NAME = "Webcam Ground"
HIGH_CAM_NAME = "Webcam High"
```

### HSV Ranges (OpenCV H=0-180)
```kotlin
POLLEN_HSV_RANGE = intArrayOf(14, 35, 100, 255, 100, 255)
NECTAR_BLUE_HSV_RANGE = intArrayOf(100, 140, 100, 255, 100, 255)
NECTAR_RED_HSV_RANGE = intArrayOf(0, 12, 100, 255, 100, 255)
NECTAR_RED_WRAP_HSV_RANGE = intArrayOf(168, 180, 100, 255, 100, 255)
FLOWER_GREEN_HSV_RANGE = intArrayOf(45, 80, 50, 255, 50, 255)
ROBOT_DARK_HSV_RANGE = intArrayOf(0, 180, 0, 120, 0, 90)
```

### Motor Gains (motors.kt uses these)
```kotlin
// Drivetrain (Yellow Jacket 117 RPM)
DRIVE_MAX_RPM = 117.0
DRIVE_POSITION_KP = 8.0

// Flywheel (6000 RPM)
FLYWHEEL_MAX_RPM = 6000.0
FLYWHEEL_PRIME_RPM = 2000.0

// Intake
INTAKE_MAX_RPM = 2500.0
```

### Autonomous Time Budget
```kotlin
AUTONOMY_PERIOD_SEC = 30.0
AUTONOMY_PARK_RESERVE_SEC = 3.0
AUTONOMY_FIRE_RESERVE_SEC = 6.0
MIN_CYCLE_REMAINING_SEC = 14.0
AUTONOMOUS_COLLECT_TIMEOUT_MS = 9000
FLOWER_PULL_TIMEOUT_MS = 6000
ALIGN_TIMEOUT_MS = 8000
```

### Collection & Dodge
```kotlin
CAPTURE_TURN_GAIN = 0.008
CAPTURE_TURN_MAX_POWER = 0.20
TARGET_SWITCH_MARGIN = 1.5
AVOID_ROBOT_BEARING_DEG = 15.0
DODGE_LATCH_MS = 600
AVOID_ROBOT_STRAFE = 0.25
PICKUP_CURRENT_BUMP_A = 1.5
PICKUP_CONFIRM_CM = 25.0
PICKUP_SETTLE_MS = 400
SEARCH_SWEEP_TURN_POWER = 0.12
```

### Shooter
```kotlin
FLYWHEEL_FEED_TIME_MS = 1200
SHOT_DIP_THRESHOLD_RPM = 250
SHOT_DETECT_WINDOW_MS = 900
SHOOT_MAX_RANGE_CM = 350.0
SHOOTER_CALIBRATION_RPM = doubleArrayOf(...)  // Fill from calibration
SHOOTER_CALIBRATION_DISTANCE_CM = doubleArrayOf(...)
```

### Vision & Pose
```kotlin
C270_FOCAL_LENGTH_PIXELS = 543.0
TELEOP_FIELD_CENTRIC_DEFAULT = false
TELEOP_POSE_HOLD_DEFAULT = true
TELEOP_INPUT_DEADZONE = 0.08
TELEOP_HOLD_POSITION_KP = 0.06
TELEOP_HOLD_HEADING_KP = 0.02
TELEOP_HOLD_MAX_POWER = 0.25
```

### WorldModel
```kotlin
WM_CONFIDENCE_TAU_SEC = 10.0
WM_TRACK_MATCH_CM = 40.0
DEAD_RECKON_MAX_AGE_SEC = 3.0
CHASE_MIN_CONTOUR_AREA_PX = 350
LAST_SLOT_PREFER_HIGHEST_VALUE = true
MAX_BALLS_HELD = 3
```

### Flower
```kotlin
FLOWER_APPROACH_CM = 25.0
FLOWER_PULL_TIMEOUT_MS = 6000
// flowerPullRoutine lambda in globals.kt — customize per event
```

### TeleOp Assists (HiveScoringConfig)
```kotlin
RANGE_GAIN = 0.004
BEARING_GAIN = 0.02
ASSIST_MAX_POWER = 0.30
CAPTURE_RANGE_GAIN = 0.02
CAPTURE_BEARING_GAIN = 0.02
```

## 13.2 MecanumDrive.Params (@Config — Dashboard Live Tuning)

```kotlin
// IMU Orientation (MEASURE PHYSICALLY)
logoFacingDirection = UP          // Logo points UP?
usbFacingDirection = FORWARD      // USB ports face FORWARD?

// Drive Model (from DrivetrainSpec)
inPerTick = ...                   // DO NOT EDIT — from DrivetrainSpec
lateralInPerTick = ...            // DO NOT EDIT
trackWidthTicks = ...             // DO NOT EDIT

// Feedforward (TUNE FIRST)
kS = 0.0
kV = 0.0
kA = 0.0

// Motion Profile
maxWheelVel = 50.0
minProfileAccel = -30.0
maxProfileAccel = 50.0
maxAngVel = 3.14159
maxAngAccel = 3.14159

// PID Gains (TUNE AFTER FF)
axialGain = 0.0
lateralGain = 0.0
headingGain = 0.0
axialVelGain = 0.0
lateralVelGain = 0.0
headingVelGain = 0.0
```

## 13.3 VisionTuning (@Config — Dashboard Live Tuning)

```kotlin
// Feature Detection
MAX_FEATURES = 60
QUALITY_LEVEL = 0.05
MIN_DISTANCE_PX = 15

// LK Tracking
LK_WIN_SIZE = 21
LK_MAX_LEVEL = 3
LK_MAX_ERROR_PX = 4.0
MIN_TRACKS = 25
REACQUIRE_INTERVAL_FRAMES = 15

// RANSAC
RANSAC_ITERATIONS = 40
RANSAC_INLIER_THRESHOLD_PX = 3.0
RANSAC_MIN_INLIERS = 12
RANSAC_MIN_INLIER_RATIO = 0.45
RANSAC_MAX_SCALE_ERROR = 0.10

// Landmarks
LANDMARK_CLUSTER_RADIUS_PX = 40
MAX_LANDMARKS = 20
MIN_LANDMARK_OBS = 8
MIN_LANDMARK_AGE_FRAMES = 15

// Fusion
FUSION_MIN_CONFIDENCE = 0.35
FUSION_MAX_GAIN = 0.20
FUSION_MAX_STEP_IN = 0.30
FUSION_MAX_STEP_DEG = 2.0
FUSION_MAX_ERR_IN = 12.0
FUSION_MAX_ERR_DEG = 15.0
FUSION_RESEED_MS = 2500

// Vision Guidance
GUIDANCE_K_FORWARD = 0.10
GUIDANCE_K_STRAFE = 0.10
GUIDANCE_K_TURN = 0.02
GUIDANCE_POS_TOL_IN = 0.75
GUIDANCE_HEADING_TOL_DEG = 3.0
GUIDANCE_SETTLE_FRAMES = 5
GUIDANCE_TIMEOUT_MS = 4000
GUIDANCE_CONFIDENCE_FLOOR = 0.45
```

## 13.4 DrivetrainSpec (Physical Measurements ONLY)

```kotlin
// MEASURE THESE — DO NOT EDIT DERIVED CONSTANTS
DRIVE_WHEEL_DIAMETER_CM = 5.0
DRIVE_MOTOR_CPR = 28
DRIVE_GEAR_RATIO = 51.46
TRACK_WIDTH_CM = 87.0
WHEELBASE_CM = 87.0
MECANUM_LATERAL_SLIP = 1.0

DEAD_WHEEL_DIAMETER_CM = 3.5
DEAD_WHEEL_TICKS_PER_REV = 2000
DEAD_WHEEL_PAR0_POS_CM = Pair(0.0, 15.2)
DEAD_WHEEL_PAR1_POS_CM = Pair(0.0, -15.2)
DEAD_WHEEL_PERP_POS_CM = Pair(-15.2, 0.0)
PAR0_REVERSED = false
PAR1_REVERSED = false
PERP_REVERSED = false
```

---

## Quick Reference Card

| Task | OpMode | Key File |
|------|--------|----------|
| Verify motor directions | Drivetrain Setup & Verify | `motors.kt` |
| Tune HSV ranges | Vision Test | `globals.kt` |
| Calibrate camera focal length | Camera Calibration | `VisionConfig.kt` |
| Tune RR drive | (Dashboard Config) | `MecanumDrive.kt` Params |
| Calibrate shooter | Flywheel Calibration | `globals.kt` shooter arrays |
| Test vision fusion | Visual Localization Test | `VisionConfig.kt` extrinsics |
| Run competition auto | BioBuzz Autonomous | `BioBuzzAutonomous.kt` |
| Run competition teleop | BioBuzz TeleOp | `BioBuzzTeleOp.kt` |

---

*Last updated: 2026–2027 season (BioBuzz). This manual covers the `main` branch as of commit d9495c9.*