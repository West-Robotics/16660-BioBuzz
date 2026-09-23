// i am used to in c++, where it is best practice to have all global variables in one file,
// so they are all easy to change and modify
package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ColorSpace
import org.opencv.core.Scalar

val controller: Gamepad = gamepad1

var DEBUG_MODE = true

const val DrivetrainFrontLeft = "0"
const val DrivetrainFrontRight = "2"
const val DrivetrainBackLeft = "1"
const val DrivetrainBackRight = "3"

const val FlywheelMotor = "flywheel"
const val IntakeMotor = "intake"
const val IntakeGearRatio = 1.0

const val WheelDiameterCm = 5.0
const val WheelGearRatio = 1.0
const val ticksPerMotorRev = 537.7
const val WheelBaseCm: Double = 87.0
const val TrackWidthCm: Double = 87.0

const val maxRPM = 2500 // this is for the mootr before gear ratio is applied

// Strategic/Pathfinding Constants (BIOBUZZ)
const val NECTAR_SCORE_MULTIPLIER = 2.0 // Nectar is worth more points than Pollen
const val ROBOT_SPEED_CM_PER_SEC = 100.0 // Placeholder: Measure actual robot speed
const val FLYWHEEL_SPINUP_SEC = 2.0 // Placeholder: informational only; spin-up runs in parallel with travel
const val FLYWHEEL_PRESPIN_BALL_COUNT = 2 // Start warming the flywheel once we hold this many balls

// ================== BIOBUZZ GAME LOGIC (tune these between matches) ==================
// Everything you'd plausibly change on game day lives HERE or is indexed below.
// Algorithm-internal tuning lives in dedicated configs, all listed so nothing
// is hard to find from this file:
//   * Shooting range window / aim gains  -> hivescoring.kt  HiveScoringConfig
//   * Vision tracking/fusion thresholds   -> vision/VisionTuning.kt
//   * Camera calibrations (MEASURE)       -> vision/VisionConfig.kt
//   * Road Runner drive model/gains      -> roadrunner/MecanumDrive.kt PARAMS
//   * AprilTag test targets + camera names-> this file (see camera section above)

// -- Ball capacity (single source of truth; StrategicController uses it too) --
const val MAX_BALLS_HELD = 3            // MEASURE: how many balls the robot can store

// -- Ball point values (keep in sync with NECTAR_SCORE_MULTIPLIER) --
const val POLLEN_VALUE = 1.0
const val NECTAR_VALUE = 2.0

// -- Ball targeting priority (autonomous collection decisions) --
// Normal slots: utility = value * PRIORITY_VALUE_WEIGHT
//                        / (1 + distanceMeters * PRIORITY_DISTANCE_WEIGHT)
// Raise VALUE_WEIGHT to chase nectar harder; raise DISTANCE_WEIGHT to prefer near balls.
const val PRIORITY_VALUE_WEIGHT = 1.0
const val PRIORITY_DISTANCE_WEIGHT = 1.0
// With exactly ONE slot left, take the highest-value ball outright (a 2x ball
// in the last slot beats a 1x ball even if somewhat farther).
const val LAST_SLOT_PREFER_HIGHEST_VALUE = true

// -- Autonomous collection behavior --
const val AUTONOMOUS_COLLECT_TIMEOUT_MS = 9000.0 // hard cap on the collection phase
const val PICKUP_CONFIRM_CM = 25.0   // ball closer than this can count as picked up
const val PICKUP_SETTLE_MS = 400.0    // must stay that close this long to confirm

// ================== FLOWER PULL-OUT — CUSTOMIZE FOR YOUR MECHANISM ==================
const val FLOWER_PULL_ENABLED = true         // autonomous tries flowers at all
const val FLOWER_APPROACH_CM = 25.0          // MEASURE: standoff taken before pulling
const val FLOWER_PULL_TIMEOUT_MS = 6000.0    // phase cap (approach + pull)
const val FLOWER_SETTLE_AFTER_PULL_MS = 600L // intake time to swallow the released ball

/**
 * ==== HOW THE ROBOT PULLS A BALL OUT OF A FLOWER — EDIT THIS ====
 *
 * Autonomous drives to FLOWER_APPROACH_CM of a flower, stops, and calls
 * this routine. Replace the body with the real mechanism sequence for the
 * final design (servo sweep, arm motion, winch, etc.). It runs ONCE per
 * flower stop, on the OpMode thread, so Thread.sleep / blocking waits are OK.
 *
 * Default placeholder: reverse-pulse the intake (assumes pulling = backing
 * the ball out with the intake until a real mechanism exists).
 */
var flowerPullRoutine: () -> Unit = {
    intake.spinUp(-1.0)
    try {
        Thread.sleep(1000)
    } catch (_: InterruptedException) {
        Thread.currentThread().interrupt()
    }
    intake.stop()
}

/** Type of ball collected, for the ball counter. UNKNOWN = source cannot identify type (e.g. beam-break sensor). */
enum class BallType { POLLEN, NECTAR, UNKNOWN }

// some experimental camera stuff

const val GROUND_CAM_NAME = "Webcam Ground"
const val HIGH_CAM_NAME = "Webcam High"

// --- Hybrid visual localization (two cameras with intentionally different views) ---
// The UP camera (ceiling: rafters, trusses, lights, beams) and the FRONT camera
// (field walls, perimeter, posts, corners, rigid structures) feed ONE pose estimator.
// These default to the webcams already present in the hardware configuration;
// change the names here if dedicated cameras are added.
const val UP_CAM_NAME = HIGH_CAM_NAME      // MEASURE: camera angled upward at the ceiling
const val FRONT_CAM_NAME = GROUND_CAM_NAME // MEASURE: camera facing forward at field structures

// MEASURE: the IMU device name in the hardware configuration ("imu" is the REV Hub default).
const val RR_IMU_NAME = "imu"
// HIGH (UP) camera resolution — must be a mode the Logitech C270 actually
// supports (640x480, 800x448, 1280x720) and must match VisionTuning's
// UP_CAM_WIDTH/HEIGHT, which the UP-camera intrinsics (VisionConfig.UP_CAMERA)
// are calibrated for. Calibrate real values with the "Camera Calibration (C270)" OpMode.
const val CAMERA_WIDTH_PIXELS = 800
const val CAMERA_HEIGHT_PIXELS = 448
const val GROUND_CAM_WIDTH = 640
const val GROUND_CAM_HEIGHT = 480
const val C270_FOCAL_LENGTH_PIXELS = 543.0
const val C270_HORIZONTAL_FOV_DEGREES = 47.0
const val POLLEN_PHYSICAL_WIDTH_MM = 71.12
const val NECTAR_PHYSICAL_WIDTH_MM = 91.44
const val APRILTAG_PHYSICAL_WIDTH_MM = 82.55 // 3.25 inches
const val FLOWER_PHYSICAL_WIDTH_MM = 101.5 // 4 inches (opening)
const val ROBOT_PHYSICAL_WIDTH_MM = 450.0 // ~18 inch robot footprint, for range estimation
// AprilTag IDs for BIOBUZZ Hives
val RED_HIVE_TAGS = listOf(30, 31, 32, 33, 34, 35, 36, 37)
val BLUE_HIVE_TAGS = listOf(38, 39, 40, 41, 42, 43, 44, 45)

// Official BIOBUZZ hive AprilTag CLUSTERS (SDK 12.0 game tag library).
// Each hive is a 4-tag CLUSTER whose origin is the CENTER OF THE CELL OPENING —
// so the cluster pose IS the scoring aim point. Match by shortName because the
// cluster member-ID accessors are not public in the SDK.
//   RED SCORING: tags 30-33    RED AUDIENCE: tags 34-37
//   BLUE AUDIENCE: tags 38-41  BLUE SCORING: tags 42-45
const val HIVE_CLUSTER_RED_SCORING_NAME = "RED SCORING"
const val HIVE_CLUSTER_RED_AUDIENCE_NAME = "RED AUDIENCE"
const val HIVE_CLUSTER_BLUE_AUDIENCE_NAME = "BLUE AUDIENCE"
const val HIVE_CLUSTER_BLUE_SCORING_NAME = "BLUE SCORING"
val HIVE_CLUSTER_NAMES = setOf(
    HIVE_CLUSTER_RED_SCORING_NAME,
    HIVE_CLUSTER_RED_AUDIENCE_NAME,
    HIVE_CLUSTER_BLUE_AUDIENCE_NAME,
    HIVE_CLUSTER_BLUE_SCORING_NAME,
)

// HSV Ranges for BIOBUZZ
val POLLEN_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(10.0, 100.0, 100.0),
    Scalar(40.0, 255.0, 255.0)
)
val NECTAR_BLUE_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(100.0, 100.0, 100.0),
    Scalar(140.0, 255.0, 255.0)
)
val NECTAR_RED_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(0.0, 100.0, 100.0),
    Scalar(10.0, 255.0, 255.0)
)
val FLOWER_GREEN_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(40.0, 50.0, 50.0),
    Scalar(80.0, 255.0, 255.0)
)
// Robots: large dark shapes (black plastic, shadows). Loose V/S bounds catch most lighting.
val ROBOT_DARK_HSV_RANGE = ColorRange(
    ColorSpace.HSV,
    Scalar(0.0, 0.0, 0.0),
    Scalar(180.0, 120.0, 90.0)
)

// Minimum contour area (px^2 at 640x480) for a blob to count as a robot, not noise.
const val MIN_ROBOT_CONTOUR_AREA = 4000.0
