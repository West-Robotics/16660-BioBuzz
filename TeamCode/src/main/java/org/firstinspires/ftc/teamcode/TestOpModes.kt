package org.firstinspires.ftc.teamcode

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.hardware.actuators.NextMotor
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.vision.CameraId
import org.firstinspires.ftc.teamcode.vision.VisionConfig
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.vision.VisionTuning
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection
import org.opencv.core.Point
import java.util.Locale
import kotlin.math.atan
import kotlin.math.sqrt

/** Vision localization, camera/flywheel calibration, and neural validation OpModes. */

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
        BlackBox.start(hardwareMap.appContext, "vision_test")
        val vision = VisionSubsystem(
            hardwareMap,
            extraUpProcessors = listOf(FrameGrabProcessor("UP")),
            extraFrontProcessors = listOf(FrameGrabProcessor("FRONT")),
        )

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
        BlackBox.stop()
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

// ============================================================================
// ============ Flywheel Calibration (was FlywheelCalibration.kt) ============
// ============================================================================


/**
 * ============ FLYWHEEL CALIBRATION — measure the shooter curve ============
 *
 * Walks the dual flywheel through 6000/5000/4000/3000/2000/1000 RPM while you
 * film each shot with a camera (a phone is fine). Then run
 * tools/flywheel_analyzer.py on the video — it tracks the ball automatically
 * and prints the measured landing distance for each RPM, formatted ready to
 * paste into globals.kt.
 *
 * HOW TO USE (full walkthrough in tools/README.md):
 *  1. Aim the robot straight down a clear runway. Put the camera SIDE-ON to
 *     the ball's flight path, and lay two tape marks on the floor a known
 *     distance apart (the analyzer uses them to convert pixels -> cm).
 *  2. Load a ball, run this OpMode, pick the RPM step (dpad UP/DOWN).
 *  3. Press A to spin the flywheel up; wait until telemetry says AT SPEED.
 *  4. Press X to FIRE ONE ball through the loader; film where it lands.
 *  5. Repeat for every RPM step (shoot in the SAME order as
 *     SHOOTER_CALIBRATION_RPM: 6000 down to 1000), then run the analyzer.
 *
 * Every shot is confirmed by the flywheel RPM dip and written to the Black
 * Box — those records are what tools/shooter_refit.py uses to keep refining
 * the shooter curve from real matches.
 */
@TeleOp(name = "Flywheel Calibration", group = "Test")
class FlywheelCalibration : LinearOpMode() {

    private val rpmSteps = SHOOTER_CALIBRATION_RPM.toList()
    private var stepIdx = 0
    private var fineRpm = 0.0
    private var spinning = false

    private lateinit var flywheel: FlyWheel
    private val loader = Loader()

    // Edge-detection memory so one button press = exactly one action.
    private var lastDpadUp = false
    private var lastDpadDown = false
    private var lastDpadLeft = false
    private var lastDpadRight = false
    private var lastA = false
    private var lastB = false
    private var lastX = false
    private var lastStallReset = false

    private fun commandedRpm(): Double =
        (rpmSteps[stepIdx] + fineRpm).coerceIn(0.0, FLYWHEEL_MOTOR_MAX_RPM)

    override fun runOpMode() {
        // boot probe: this OpMode exists to test the shooter + loader — a
        // single flywheel motor is enough for bench testing.
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.anyFlywheel || !hw.loaderServo) {
            while (!isStopRequested) {
                telemetry.addLine("!! SHOOTER HARDWARE NOT DETECTED !!")
                telemetry.addLine("Calibration needs at least one flywheel motor + the loader servo.")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
                telemetry.update()
                sleep(50) // don't busy-spin a core during INIT
            }
            return
        }

        BlackBox.start(hardwareMap.appContext, "calib")
        hw.logToBlackBox()
        flywheel = FlyWheel(hasLeft = hw.flywheelLeft, hasRight = hw.flywheelRight)
        loader.home()
        while (!isStarted && !isStopRequested) {
            telemetry.addLine("FLYWHEEL CALIBRATION (${rpmSteps.size} RPM steps)")
            telemetry.addLine("DPAD UP/DOWN: pick RPM step   DPAD LEFT/RIGHT: fine +/-25")
            telemetry.addLine("A: toggle spin   B: stop   X: FIRE one ball (loader)")
            telemetry.addData("RPM now", String.format(Locale.US, "%.0f", rpmSteps[stepIdx]))
            telemetry.addData("Curve", ShooterModel.summary())
            telemetry.update()
        }

        while (opModeIsActive()) {
            NextMotor.motorEventLoop.poll()
            flywheel.update()
            if (gamepad1.back && !lastStallReset) flywheel.resetStall()
            lastStallReset = gamepad1.back
            if (gamepad1.dpad_up && !lastDpadUp && stepIdx < rpmSteps.size - 1) {
                stepIdx++; fineRpm = 0.0
            }
            if (gamepad1.dpad_down && !lastDpadDown && stepIdx > 0) {
                stepIdx--; fineRpm = 0.0
            }
            if (gamepad1.dpad_right && !lastDpadRight) fineRpm += 25.0
            if (gamepad1.dpad_left && !lastDpadLeft) fineRpm -= 25.0
            if (gamepad1.a && !lastA) spinning = !spinning
            if (gamepad1.b && !lastB) spinning = false

            val rpm = commandedRpm()
            if (spinning) flywheel.spinToRpm(rpm) else flywheel.stop()

            // Fire exactly one ball: a single loader stroke, CONFIRMED by the
            // flywheel RPM dip, then logged — these records are the raw data
            // for the shooter curve (tools/shooter_refit.py + video analyzer).
            if (gamepad1.x && !lastX && spinning && !loader.busy &&
                loader.phase == LoaderPhase.HOME
            ) {
                loader.stroke()
                while (opModeIsActive() && loader.busy) {
                    NextMotor.motorEventLoop.poll()
                    flywheel.update()
                    telemetry.addData("LOADER", "stroking...")
                    telemetry.update()
                }
                val detector = ShotDetector(flywheel)
                detector.arm()
                while (opModeIsActive() && detector.update() == ShotDetector.State.ARMED) {
                    NextMotor.motorEventLoop.poll()
                    flywheel.update()
                    telemetry.addData(
                        "SHOT WATCH",
                        "%.0f RPM (need a %.0f RPM dip)".format(
                            detector.measuredRpm(), SHOT_DIP_THRESHOLD_RPM,
                        ),
                    )
                    telemetry.update()
                }
                loader.retract()
                BlackBox.event(
                    "calibration_shot",
                    "commandedRpm" to rpm,
                    "dipRpm" to detector.lastDipRpm,
                    "result" to detector.state.name,
                )
            }

            if (flywheel.stalled) {
                telemetry.addLine("!! ${flywheel.stallSide} STALLED — motors OFF; clear jam, press Back !!")
            }
            telemetry.addData(
                "Current L/R (A)",
                "%.1f / %.1f".format(flywheel.leftCurrentAmps() ?: -1.0, flywheel.rightCurrentAmps() ?: -1.0),
            )
            telemetry.addData("Commanded", String.format(Locale.US, "%.0f RPM", rpm))
            telemetry.addData(
                "Measured L/R",
                String.format(Locale.US, "%.0f / %.0f RPM", flywheel.measuredLeftRpm(), flywheel.measuredRightRpm()),
            )
            telemetry.addData("AT SPEED", flywheel.isAtSpeed(rpm))
            telemetry.addData(
                "Step", "%d/%d (fine %+.0f)".format(stepIdx + 1, rpmSteps.size, fineRpm),
            )
            telemetry.addLine(
                "FILM THIS SHOT, then note: " +
                    String.format(Locale.US, "%.0f", rpm) + " RPM -> ? cm",
            )
            telemetry.update()

            lastDpadUp = gamepad1.dpad_up
            lastDpadDown = gamepad1.dpad_down
            lastDpadLeft = gamepad1.dpad_left
            lastDpadRight = gamepad1.dpad_right
            lastA = gamepad1.a
            lastB = gamepad1.b
            lastX = gamepad1.x
        }

        flywheel.stop(breakMotor = true)
        loader.home()
        NextMotor.motorEventLoop.poll()
        BlackBox.stop()
    }
}

// ============================================================================
// ============ Neural Net Test (was NnTestOpMode.kt) ============
// ============================================================================


/**
 * ============ NEURAL NET TEST — trust-building bench ============
 *
 * Runs both on-robot neural nets against the live cameras WITHOUT any robot
 * motion, so you can validate them on the bench:
 *
 *  * the ball DETECTOR's verdicts (boxes are also drawn cyan on the DS
 *    preview by the processor itself), with the current authority mode,
 *  * the SHOT PREDICTOR's P(hit) across a range of distances at the RPM the
 *    calibrated curve commands, at the live battery voltage.
 *
 * Interpretation: the predictor table should look like a sensible curve —
 * high P(hit) where the shooter is dialed in, dropping off at the extremes.
 * If it's flat or weird, the training data or model is bad; retrain.
 */
@TeleOp(name = "Neural Net Test", group = "Test")
class NnTestOpMode : LinearOpMode() {

    override fun runOpMode() {
        // boot probe: without the cameras this OpMode has nothing to test
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.vision) {
            while (!isStopRequested) {
                telemetry.addLine("!! AUTONOMOUS HARDWARE NOT DETECTED !!")
                telemetry.addLine("Neural Net Test needs both cameras — nothing to test.")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
                telemetry.update()
                sleep(50) // don't busy-spin a core during INIT
            }
            return
        }

        ShotPredictor.ensureLoaded(hardwareMap.appContext)
        val ballLocator = BallLocator()
        val detector = DetectorProcessor(hardwareMap.appContext)
        val vision = VisionSubsystem(
            hardwareMap,
            extraUpProcessors = emptyList(),
            extraFrontProcessors = ballLocator.processors + detector,
        )

        telemetry.addData("Detector", if (detector.loaded) "model loaded" else "no model file")
        telemetry.addData("Detector mode", NN_DETECTOR_MODE.name)
        telemetry.addData("Shot predictor", if (ShotPredictor.available) "loaded" else "not loaded")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            vision.periodic()

            val targets = ballLocator.visibleTargets()
            val nn = NeuralReferee.latest()

            telemetry.addData("NN mode", NN_DETECTOR_MODE.name)
            telemetry.addData(
                "NN detections",
                "%d %s".format(
                    nn.size,
                    if (NeuralReferee.fresh()) "(fresh)" else "(stale)",
                ),
            )
            for (d in nn) {
                telemetry.addData(
                    "  NN %s".format(d.label),
                    String.format(Locale.US, "%.0f%% @ (%.0f,%.0f)", d.confidence * 100, d.x, d.y),
                )
            }
            telemetry.addData("HSV targets (post-referee)", targets.size)
            for (t in targets.take(4)) {
                telemetry.addData(
                    "  HSV %s".format(t.label),
                    String.format(Locale.US, "%.0fcm %+.1fdeg", t.distanceCm, t.bearingDeg),
                )
            }

            if (ShotPredictor.available) {
                telemetry.addData("Battery", "%.2fV".format(ShotPredictor.batteryVolts()))
                for (distCm in listOf(90.0, 110.0, 130.0, 150.0)) {
                    val rpm = ShooterModel.rpmForDistance(distCm)
                    val p = ShotPredictor.pHit(distCm, rpm, ShotPredictor.batteryVolts())
                    telemetry.addData(
                        "P(hit) %dcm".format(distCm.toInt()),
                        "rpm %.0f -> %s".format(rpm, p?.let { "%.0f%%".format(it * 100) } ?: "?"),
                    )
                }
            }
            telemetry.update()
        }

        vision.close()
    }
}
