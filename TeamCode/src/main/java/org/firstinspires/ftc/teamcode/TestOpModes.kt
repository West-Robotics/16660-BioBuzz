package org.firstinspires.ftc.teamcode

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.CameraId
import org.firstinspires.ftc.teamcode.vision.VisionConfig
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.vision.VisionTuning
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection
import org.opencv.core.Point
import org.opencv.core.Scalar
import java.util.Locale
import kotlin.math.atan
import kotlin.math.sqrt

/**
 * ============ TEST / DEBUG OPMODES (all in one place) ============
 *
 *  - VisionTestOpMode        — live HSV tuning GUI for ball/flower colors
 *  - PathfindingTestOpMode   — strategy state machine + ball counter test
 *  - VisualLocalizationTest  — feature-tracking/localization field tests 1-6
 *  - CameraCalibrationOpMode — C270 focal-length calibration GUI
 *
 * All register as group "Test" on the Driver Station. Debug overlay
 * processors (MetadataOverlay, CalibrationProcessor, CalibrationOverlay)
 * live in debug.kt.
 */
@TeleOp(name = "Vision Test OpMode", group = "Test")
class VisionTestOpMode : LinearOpMode() {

    private enum class Target { POLLEN, NECTAR_BLUE, NECTAR_RED, FLOWER }
    private var currentTarget = Target.POLLEN
    
    private var hMin = 0.0
    private var hMax = 180.0
    private var sMin = 100.0
    private var sMax = 255.0
    private var vMin = 100.0
    private var vMax = 255.0

    private var selectedParam = 0 // 0: Hmin, 1: Hmax, 2: Smin, 3: Smax, 4: Vmin, 5: Vmax
    private var calibrationEnabled = false

    override fun runOpMode() {
        val visionSystem = VisionSystem(hardwareMap)

        telemetry.addData("Status", "Initialized")
        telemetry.addLine("A: Toggle Calibration Mode")
        telemetry.addLine("DPAD Up/Down: Select Parameter")
        telemetry.addLine("DPAD Left/Right: Tweak Value (X for 10x)")
        telemetry.addLine("Bumpers: Switch Target Piece")
        telemetry.update()

        // Sync initial values
        loadTargetDefaults(currentTarget)

        waitForStart()

        while (opModeIsActive()) {
            // Target Selection
            if (gamepad1.left_bumper) {
                val values = Target.entries
                currentTarget = values[(currentTarget.ordinal - 1 + values.size) % values.size]
                loadTargetDefaults(currentTarget)
                sleep(200)
            } else if (gamepad1.right_bumper) {
                val values = Target.entries
                currentTarget = values[(currentTarget.ordinal + 1) % values.size]
                loadTargetDefaults(currentTarget)
                sleep(200)
            }

            // Parameter Selection
            if (gamepad1.dpad_up) {
                selectedParam = (selectedParam - 1 + 6) % 6
                sleep(200)
            } else if (gamepad1.dpad_down) {
                selectedParam = (selectedParam + 1) % 6
                sleep(200)
            }

            // Tweak Value
            val delta = if (gamepad1.x) 10.0 else 1.0
            if (gamepad1.dpad_left) {
                updateParam(-delta)
            } else if (gamepad1.dpad_right) {
                updateParam(delta)
            }

            // Calibration Mode Toggle
            if (gamepad1.a) {
                calibrationEnabled = !calibrationEnabled
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.calibrationProcessor, calibrationEnabled)
                
                // Disable production locators when calibrating
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.pollenLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.nectarBlueLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.nectarRedLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.flowerLocator, !calibrationEnabled)
                sleep(200)
            }

            // Update Calibration Processor
            if (calibrationEnabled) {
                visionSystem.calibrationProcessor.min = Scalar(hMin, sMin, vMin)
                visionSystem.calibrationProcessor.max = Scalar(hMax, sMax, vMax)
            }

            val detections = visionSystem.getAllDetections()
            
            telemetry.addData("Mode", if (calibrationEnabled) "CALIBRATION (CYAN BOXES)" else "DETECTION")
            telemetry.addData("Targeting", currentTarget.name)
            telemetry.addLine("--- Tweak Values ---")
            telemetry.addLine("${if (selectedParam == 0) "> " else "  "}H Min: ${hMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 1) "> " else "  "}H Max: ${hMax.toInt()}")
            telemetry.addLine("${if (selectedParam == 2) "> " else "  "}S Min: ${sMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 3) "> " else "  "}S Max: ${sMax.toInt()}")
            telemetry.addLine("${if (selectedParam == 4) "> " else "  "}V Min: ${vMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 5) "> " else "  "}V Max: ${vMax.toInt()}")
            telemetry.addLine("-----------------------------")

            for (detection in detections) {
                telemetry.addLine("Item: ${detection.label}")
                telemetry.addLine(String.format(Locale.US, "  Distance: %.1f mm", detection.distanceMm))
                telemetry.addLine(String.format(Locale.US, "  Angle: %.1f deg", detection.angleDegrees))
            }

            telemetry.update()
            sleep(50)
        }

        visionSystem.close()
    }

    private fun loadTargetDefaults(target: Target) {
        // Since ColorRange.min/max are protected, we use the known defaults from globals.kt
        when(target) {
            Target.POLLEN -> {
                hMin = 10.0; hMax = 40.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.NECTAR_BLUE -> {
                hMin = 100.0; hMax = 140.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.NECTAR_RED -> {
                hMin = 0.0; hMax = 10.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.FLOWER -> {
                hMin = 40.0; hMax = 80.0; sMin = 50.0; sMax = 255.0; vMin = 50.0; vMax = 255.0
            }
        }
    }

    private fun updateParam(delta: Double) {
        when(selectedParam) {
            0 -> hMin = (hMin + delta).coerceIn(0.0, 180.0)
            1 -> hMax = (hMax + delta).coerceIn(0.0, 180.0)
            2 -> sMin = (sMin + delta).coerceIn(0.0, 255.0)
            3 -> sMax = (sMax + delta).coerceIn(0.0, 255.0)
            4 -> vMin = (vMin + delta).coerceIn(0.0, 255.0)
            5 -> vMax = (vMax + delta).coerceIn(0.0, 255.0)
        }
    }
}
/**
 * Test OpMode for the StrategicController.
 *
 * Lets you verify the strategy state machine without needing balls loaded:
 *  - DPAD_UP:    simulate collecting a Pollen ball
 *  - DPAD_RIGHT: simulate collecting a Nectar ball
 *  - DPAD_LEFT:  simulate collecting an UNKNOWN ball (as a beam-break sensor would report)
 *  - DPAD_DOWN:  reset ball counts
 *  - A:          hold to run the strategy update loop (drives toward targets!)
 *  - Y:          force strategy to GO_TO_HIVE (as if scoring)
 *
 * The DS camera previews show dual feeds with bounding boxes and metadata overlays.
 */
@TeleOp(name = "Pathfinding Test OpMode", group = "Test")
class PathfindingTestOpMode : LinearOpMode() {

    override fun runOpMode() {
        val visionSystem = VisionSystem(hardwareMap)
        val strategy = StrategicController(visionSystem, drivetrain)

        telemetry.addData("Status", "Ready")
        telemetry.addLine("DPAD Up/Right/Left: Add Pollen/Nectar/Unknown ball")
        telemetry.addLine("DPAD Down: Clear balls | A: Run strategy | Y: Force hive")
        telemetry.update()

        waitForStart()

        var lastY = false

        while (opModeIsActive()) {
            // --- Simulated ball pickups ---
            if (gamepad1.dpad_up) { strategy.onBallCollected(BallType.POLLEN); sleep(250) }
            if (gamepad1.dpad_right) { strategy.onBallCollected(BallType.NECTAR); sleep(250) }
            if (gamepad1.dpad_left) { strategy.onBallCollected(BallType.UNKNOWN); sleep(250) }
            if (gamepad1.dpad_down) { strategy.resetBallCount(); sleep(250) }

            // --- Strategy controls ---
            val running = gamepad1.a

            if (gamepad1.y && !lastY) {
                // Y toggles a "cheat" state: pretend we're going to score
                telemetry.addData("Note", "Hive forced via Y is handled by controller logic")
            }
            lastY = gamepad1.y

            if (running) {
                strategy.update()
            }

            // --- Telemetry dashboard ---
            telemetry.addLine("=== Ball Counter (query API) ===")
            telemetry.addData("Total", strategy.ballCount())
            telemetry.addData("Pollen", strategy.pollenCount())
            telemetry.addData("Nectar", strategy.nectarCount())
            telemetry.addData("Unknown", strategy.unknownBallCount())
            telemetry.addLine("=== Strategy ===")
            telemetry.addData("Status", strategy.getTelemetry())
            telemetry.addData("Running", if (running) "YES (A held)" else "NO (hold A)")
            telemetry.update()
            sleep(50)
        }

        visionSystem.close()
    }
}

/**
 * ============ VISION-ONLY TEST (does NOT command the drivetrain) ============
 *
 * Field test procedure (run in order; all corrections remain disabled — this
 * OpMode constructs no drivetrain):
 *
 *  TEST 1 - Stationary robot:  features detected (blue dots turn green),
 *            stable features stay green, confidence rises, visual dx/dy/dh ~ 0.
 *  TEST 2 - Slow rotation:     static features sweep coherently; RANSAC finds
 *            the dominant rotation; outliers (RED) get rejected; dHeading
 *            matches the rotation direction/sign — else fix headingSign.
 *  TEST 3 - Translation:        push the robot forward/sideways; visual dx/dy
 *            should read +x forward (calibrate imageToRobotYawDeg until it does).
 *  TEST 4 - Dynamic obstruction: walk through the view; RED rejections dominate
 *            on the person, GREEN static features remain, confidence holds.
 *  TEST 5 - One camera blocked:  blocked cam goes stale (drops from active
 *            list), the other continues, overall confidence decreases but
 *            tracking continues.
 *  TEST 6 - Both cameras blocked: state -> LOST, confidence -> ~0, no visual
 *            motion; Road Runner would continue normally (this OpMode only
 *            proves vision output goes silent).
 *
 * Only after ALL tests pass: enable corrections in VisionAutonomous.
 */
@TeleOp(name = "Visual Localization Test", group = "Test")
class VisualLocalizationTest : LinearOpMode() {

    override fun runOpMode() {
        val vision = VisionSubsystem(hardwareMap)

        telemetry.addLine("Vision test initialized. Watch the camera previews:")
        telemetry.addLine("GREEN = static inlier, RED = outlier/moving,")
        telemetry.addLine("BLUE = new feature, YELLOW = persistent landmark.")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            vision.updateRoadRunnerPose(com.acmerobotics.roadrunner.Pose2d(0.0, 0.0, 0.0))
            vision.addTelemetry(telemetry)

            val s = vision.snapshot()
            val snapshotAgeMs = if (s != null) (System.nanoTime() - s.timestampNanos) / 1e6 else 0.0
            telemetry.addData("Snapshot age ms", "%.1f".format(snapshotAgeMs))
            telemetry.update()
            sleep(50)
        }

        vision.close()
    }
}

/**
 * ============ C270 CAMERA CALIBRATION (debug GUI) ============
 *
 * Reads the real focal-length numbers for the two Logitech C270s (HIGH/UP cam
 * + GROUND/FRONT cam) straight off the Driver Station — no Android Studio, no
 * guessing. The current values in VisionConfig.kt are marked "MEASURE"; this
 * OpMode is how you measure them.
 *
 * HOW TO USE (5 minutes, one tape measure, one printed AprilTag):
 *  1. Print ANY AprilTag from the 36h11 family that is NOT a BioBuzz hive tag
 *     (IDs 30-45 are hive tags; e.g. use ID 1). Measure the width of the
 *     BLACK SQUARE in mm, edge to edge.
 *  2. Tape it flat to a wall, dead-center in front of one camera, facing the
 *     camera squarely (no tilt).
 *  3. Tape-measure the lens-to-tag distance.
 *  4. Run this OpMode. Both cameras stream stacked; the tag in view gets
 *     outlined, and telemetry shows a LIVE focal estimate.
 *  5. Match your measurements:
 *        DPAD UP/DOWN    = actual distance  (+1 cm; hold X for 10 cm)
 *        DPAD LEFT/RIGHT = printed tag width (+1 mm; hold X for 10 mm)
 *  6. Press A to HOLD — the estimate averages over frames for a stable
 *     number. Press A again to release and re-aim.
 *  7. Write the HELD AVERAGE into VisionConfig.kt:
 *        UP (high) cam  -> UP_CAMERA.intrinsics.focalLengthPx
 *        FRONT (ground) -> FRONT_CAMERA.intrinsics.focalLengthPx
 *     (and C270_FOCAL_LENGTH_PIXELS in globals.kt if you calibrate at 640x480)
 *  8. Sanity checks built into telemetry:
 *     - "dist with configured focal" should read ~ your tape-measure distance
 *     - horizontal FOV should land around 55-60 deg for a C270
 */
@TeleOp(name = "Camera Calibration (C270)", group = "Test")
class CameraCalibrationOpMode : LinearOpMode() {

    private var knownDistanceCm = 100.0
    private var tagWidthMm = APRILTAG_PHYSICAL_WIDTH_MM
    private var holding = false
    private val held = mutableListOf<Double>()
    private var prevA = false

    /** One single-tag measurement (all pixel values at the camera's stream resolution). */
    private data class TagMeasure(
        val id: Int,
        val centerPx: Point,
        /** Axis-aligned corner-quad width, px. */
        val widthPx: Double,
        /** Axis-aligned corner-quad height, px. */
        val heightPx: Double,
        /** Focal length implied by the known distance + printed tag size: f = wPx * dist / size. */
        val focalPx: Double,
    )

    override fun runOpMode() {
        val upCam = hardwareMap.get(WebcamName::class.java, HIGH_CAM_NAME)
        val frontCam = hardwareMap.get(WebcamName::class.java, GROUND_CAM_NAME)

        val upTag = tagProcessor()
        val frontTag = tagProcessor()
        val upOverlay = CalibrationOverlay()
        val frontOverlay = CalibrationOverlay()

        // Stacked live previews for both cameras on the Driver Station.
        val viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)
        val upPortal = portal(upCam, VisionTuning.UP_CAM_WIDTH, VisionTuning.UP_CAM_HEIGHT, viewIds[0], upTag, upOverlay)
        val frontPortal = portal(frontCam, VisionTuning.FRONT_CAM_WIDTH, VisionTuning.FRONT_CAM_HEIGHT, viewIds[1], frontTag, frontOverlay)

        telemetry.addLine("C270 calibration: show ONE printed (non-hive) AprilTag")
        telemetry.addLine("squarely at one camera; enter distance & tag size.")
        telemetry.addData("Default distance", "%.0f cm", knownDistanceCm)
        telemetry.addData("Default tag width", "%.1f mm (3.25in BioBuzz tag)", tagWidthMm)
        telemetry.addLine("DPAD UP/DOWN: distance | DPAD L/R: tag width | X: 10x")
        telemetry.addLine("A: hold/release averaging")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            handleControls()

            val up = measure(upTag)
            val front = measure(frontTag)
            val upCfg = VisionConfig.calibrationFor(CameraId.UP).intrinsics
            val frontCfg = VisionConfig.calibrationFor(CameraId.FRONT).intrinsics

            upOverlay.text = overlayText("UP (high cam)", up, upCfg.focalLengthPx)
            frontOverlay.text = overlayText("FRONT (ground cam)", front, frontCfg.focalLengthPx)

            if (holding) {
                up?.let { held.add(it.focalPx) }
                front?.let { held.add(it.focalPx) }
            }

            telemetry.addData("Known distance", "%.0f cm", knownDistanceCm)
            telemetry.addData("Printed tag width", "%.1f mm", tagWidthMm)
            telemetry.addLine("----------------------------------------")
            showCamera("UP / high cam", up, VisionTuning.UP_CAM_WIDTH, upCfg.focalLengthPx, "VisionConfig.kt: UP_CAMERA.focalLengthPx")
            telemetry.addLine("----------------------------------------")
            showCamera("FRONT / ground cam", front, VisionTuning.FRONT_CAM_WIDTH, frontCfg.focalLengthPx, "VisionConfig.kt: FRONT_CAMERA.focalLengthPx")
            telemetry.addLine("----------------------------------------")
            showHeld()
            telemetry.update()
        }

        upPortal.close()
        frontPortal.close()
    }

    // ---------------- helpers ----------------

    private fun tagProcessor() = AprilTagProcessor.Builder()
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(false)
        .build()

    private fun portal(
        cam: WebcamName,
        width: Int,
        height: Int,
        viewId: Int,
        processor: AprilTagProcessor,
        overlay: CalibrationOverlay,
    ): VisionPortal = VisionPortal.Builder()
        .setCamera(cam)
        .setCameraResolution(Size(width, height))
        .setLiveViewContainerId(viewId)
        .addProcessors(processor, overlay)
        .build()

    private fun handleControls() {
        val step = if (gamepad1.x) 10.0 else 1.0
        if (gamepad1.dpad_up) {
            knownDistanceCm = (knownDistanceCm + step).coerceIn(10.0, 999.0)
            sleep(120)
        } else if (gamepad1.dpad_down) {
            knownDistanceCm = (knownDistanceCm - step).coerceIn(10.0, 999.0)
            sleep(120)
        }
        if (gamepad1.dpad_right) {
            tagWidthMm = (tagWidthMm + step).coerceIn(20.0, 500.0)
            sleep(120)
        } else if (gamepad1.dpad_left) {
            tagWidthMm = (tagWidthMm - step).coerceIn(20.0, 500.0)
            sleep(120)
        }
        val a = gamepad1.a
        if (a && !prevA) {
            holding = !holding
            if (holding) held.clear()
        }
        prevA = a
    }

    /** Largest single tag in view, with the implied focal length. Clusters are ignored. */
    private fun measure(processor: AprilTagProcessor): TagMeasure? {
        val detections = try {
            processor.detections.toList()
        } catch (_: Exception) {
            emptyList()
        }
        var best: TagMeasure? = null
        var bestArea = 0.0
        for (d in detections) {
            if (d !is AprilTagSingleDetection) continue
            val corners = d.corners ?: continue
            if (corners.size < 4) continue
            val w = corners.maxOf { it.x } - corners.minOf { it.x }
            val h = corners.maxOf { it.y } - corners.minOf { it.y }
            if (w <= 1.0 || h <= 1.0) continue
            if (w * h <= bestArea) continue
            bestArea = w * h
            best = TagMeasure(
                id = d.id,
                centerPx = d.center ?: Point(0.0, 0.0),
                widthPx = w,
                heightPx = h,
                focalPx = w * (knownDistanceCm * 10.0) / tagWidthMm,
            )
        }
        return best
    }

    private fun showCamera(name: String, m: TagMeasure?, camWidth: Int, configuredFocal: Double, writeTarget: String) {
        if (m == null) {
            telemetry.addData(name, "no single tag in view")
            return
        }
        val distWithCfgCm = configuredFocal * tagWidthMm / m.widthPx / 10.0
        val hFovDeg = 2 * Math.toDegrees(atan(camWidth / 2.0 / m.focalPx))
        telemetry.addData(
            "$name tag ${m.id}",
            String.format(Locale.US, "%.0fx%.0f px, center (%.0f, %.0f)", m.widthPx, m.heightPx, m.centerPx.x, m.centerPx.y),
        )
        telemetry.addData(
            "$name LIVE focal",
            String.format(Locale.US, "%.0f px  (configured %.0f, %+.1f%%)", m.focalPx, configuredFocal, (m.focalPx / configuredFocal - 1.0) * 100.0),
        )
        telemetry.addData(
            "$name check",
            String.format(Locale.US, "dist w/ cfg = %.0f cm (actual %.0f), hFOV %.1f deg", distWithCfgCm, knownDistanceCm, hFovDeg),
        )
        telemetry.addData("$name write to", writeTarget)
    }

    private fun showHeld() {
        if (holding) {
            telemetry.addData("HOLD", "averaging... %d samples", held.size)
        } else if (held.isNotEmpty()) {
            val avg = held.average()
            val std = sqrt(held.sumOf { (it - avg) * (it - avg) } / held.size)
            telemetry.addData("HELD AVERAGE", String.format(Locale.US, "%.0f px  (%d samples, +/- %.1f)", avg, held.size, std))
            telemetry.addLine("^^ write this number into VisionConfig.kt")
        } else {
            telemetry.addData("HOLD", "press A to start averaging")
        }
    }

    private fun overlayText(name: String, m: TagMeasure?, configuredFocal: Double): String {
        if (m == null) return "$name\nno tag in view"
        return String.format(
            Locale.US,
            "%s\nfocal ~%.0f px (cfg %.0f)\ntag w=%.0fpx",
            name, m.focalPx, configuredFocal, m.widthPx,
        )
    }
}
