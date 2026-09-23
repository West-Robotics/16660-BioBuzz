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
const val CAMERA_WIDTH_PIXELS = 1500
const val CAMERA_HEIGHT_PIXELS = 750
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
