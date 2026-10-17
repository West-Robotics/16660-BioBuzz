package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * ============ BIOBUZZ TELEOP with vision assists ============
 *
 * Driver keeps full control at all times; assists ADD bounded corrections:
 *
 *   LEFT STICK drive / RIGHT STICK turn            (existing Drivetrain)
 *   A (hold)  intake in          B (hold)  intake reverse
 *   RIGHT TRIGGER (hold)         flywheel spins to the RPM the current hive
 *                                distance needs (manual fire; full speed when
 *                                no tag is visible or the curve is uncalibrated)
 *   LEFT BUMPER (hold)           BALL CAPTURE ASSIST — strafes toward the
 *                                nearest pollen/nectar blob and creeps to a
 *                                CAPTURE_APPROACH_CM standoff.
 *   RIGHT BUMPER (hold)          SHOOTING ALIGN ASSIST — auto-turns toward the
 *                                hive AprilTag and holds the shooting window;
 *                                the flywheel spins to the calibrated RPM for
 *                                the live distance and auto-feeds balls the
 *                                moment BOTH wheels are at speed.
 *   (idle)                       flywheel stays PRIMED at FLYWHEEL_PRIME_RPM
 *                                (see globals.kt) so spin-up to a shot is quick.
 *
 * The DS preview shows the hive distance/angle live (TagInfoOverlay) and the
 * ball blobs; telemetry mirrors everything.
 */
@TeleOp(name = "BioBuzz TeleOp", group = "TeleOp")
class BioBuzzTeleOp : LinearOpMode() {

    // loader fire cycle (advances one tick at a time in the main loop)
    private enum class LoadState { IDLE, STROKING, WATCHING, RETRACTING }
    private var loadState = LoadState.IDLE

    override fun runOpMode() {
        // ---- BOOT HARDWARE PROBE: degrade gracefully if anything's missing ----
        val hw = HardwareProbe.probe(hardwareMap)
        val manualOnly = !hw.vision

        val drivetrain = if (hw.driveMotors) Drivetrain() else null

        // READ-ONLY localizer (field-centric + pose hold): the same
        // MecanumDrive the autonomous uses — dead wheels when configured,
        // drive-encoder odometry otherwise, IMU always. Constructed purely
        // for its pose; Road Runner never commands motors in this OpMode.
        // Skipped in manual-only / missing-drivetrain degraded modes.
        val locator: MecanumDrive? =
            if (hw.driveMotors) try {
                MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
            } catch (_: Exception) {
                null
            } else {
                null
            }
        var fieldCentric = TELEOP_FIELD_CENTRIC_DEFAULT
        var poseHold = TELEOP_POSE_HOLD_DEFAULT
        var stillPose: Pose2d? = null
        var lastLeftStickBtn = false
        var lastRightStickBtn = false
        val intake = Intake(hw.intakeMotor)
        val flywheel = FlyWheel(hasLeft = hw.flywheelLeft, hasRight = hw.flywheelRight)
        val loader = Loader().also { it.home() }
        val shotDetector = ShotDetector(flywheel)

        // The vision stack (portals, processors, hive estimator) is ONLY built
        // when both cameras exist in the configuration. Without it we run
        // MANUAL ONLY mode: no assists, no hive data — just drive + whatever
        // subsystems (intake/flywheel/loader) are actually configured.
        var vision: VisionSubsystem? = null
        var hive: HiveScoring? = null
        var tagOverlay: TagInfoOverlay? = null
        var ballLocator: BallLocator? = null
        if (hw.vision) {
            val aprilTagProcessor = AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(false)
                .build()
            tagOverlay = TagInfoOverlay()
            ballLocator = BallLocator()
            val nnDetector = DetectorProcessor(hardwareMap.appContext)
            vision = VisionSubsystem(
                hardwareMap,
                extraUpProcessors = listOf(
                    aprilTagProcessor, tagOverlay, FrameGrabProcessor("UP"),
                ),
                extraFrontProcessors = ballLocator.processors +
                    FrameGrabProcessor("FRONT") + nnDetector,
            )
            hive = HiveScoring(aprilTagProcessor)
        }
        val cfg = HiveScoringConfig

        // ---- Black Box (full-match recorder) + SHADOW PLANNER ----
        // The shadow planner watches the human drive and logs what IT would
        // do each second — pure observation, it never touches the motors.
        BlackBox.start(hardwareMap.appContext, "teleop")
        hw.logToBlackBox()
        ShotPredictor.ensureLoaded(hardwareMap.appContext)
        val shadowWorld = WorldModel()
        val shadowPlanner = StrategicPlanner()
        val tiltGuard = TiltGuard(hardwareMap)
        var clockStart = -1.0
        var lastShadowNanos = 0L
        var lastLogNanos = 0L

        // ---- INIT: pick your alliance before the match starts ----
        // D-pad LEFT = RED, D-pad RIGHT = BLUE. HiveScoring only ever targets
        // the selected alliance's hives, so this must match the match sheet.
        while (!isStarted && !isStopRequested) {
            if (gamepad1.dpad_left) SELECTED_TEAM = TeamColor.RED
            else if (gamepad1.dpad_right) SELECTED_TEAM = TeamColor.BLUE

            telemetry.addLine("BioBuzz TeleOp: LB=capture assist  RB=shoot assist")
            telemetry.addData("ALLIANCE", SELECTED_TEAM.name)
            telemetry.addLine("Select with D-pad: LEFT = RED, RIGHT = BLUE")
            if (manualOnly) {
                telemetry.addLine("")
                telemetry.addLine("!! AUTONOMOUS HARDWARE NOT DETECTED !!")
                telemetry.addLine("-> falling back to MANUAL ONLY mode")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
            }
            telemetry.update()
        }

        clockStart = runtime
        BlackBox.event("match_start", "alliance" to SELECTED_TEAM.name)

        while (opModeIsActive()) {
            // ---- driver input ----
            var forward = (-gamepad1.left_stick_y).toDouble()
            var strafe = gamepad1.left_stick_x.toDouble()
            var turn = gamepad1.right_stick_x.toDouble()

            // ---- localization + drive-mode toggles (edge-detected) ----
            locator?.updatePoseEstimate()
            if (gamepad1.left_stick_button && !lastLeftStickBtn) fieldCentric = !fieldCentric
            if (gamepad1.right_stick_button && !lastRightStickBtn) poseHold = !poseHold
            lastLeftStickBtn = gamepad1.left_stick_button
            lastRightStickBtn = gamepad1.right_stick_button

            if (fieldCentric && locator != null) {
                // Rotate the driver's WORLD-frame stick intent into the robot
                // frame (RR heading: CCW, radians).
                val h = locator.pose.heading.toDouble()
                val f2 = forward * cos(h) + strafe * sin(h)
                val s2 = -forward * sin(h) + strafe * cos(h)
                forward = f2
                strafe = s2
            }

            // ---- latest hive measurement (vision-dependent; null in manual-only) ----
            val target = hive?.update()
            tagOverlay?.publish(target)

            // ---- SHOOTING ALIGN ASSIST (hold RIGHT bumper) ----
            val alignAssist = gamepad1.right_bumper
            if (alignAssist && hive != null) {
                val (f, t) = hive.rangeCorrection(target)
                forward = (forward + f).coerceIn(-1.0, 1.0)
                turn = (turn + t).coerceIn(-1.0, 1.0)
            }

            // ---- flywheel: DISTANCE-BASED shooting ----
            // The commanded speed comes from the calibrated RPM-vs-distance
            // curve evaluated at the live hive distance (full speed when
            // uncalibrated or no tag is visible). While idle the wheels stay
            // PRIMED at FLYWHEEL_PRIME_RPM so spin-up to any shot is quick.
            val manualFire = gamepad1.right_trigger > 0.3
            val assistFire = alignAssist && hive?.zoneOf(target) == ScoringZone.ALIGNED
            val shootingRpm = if (flywheel.anyMotor) {
                ShotPredictor.adjustRpm(
                    hive?.shootingRpmFor(target) ?: FLYWHEEL_MOTOR_MAX_RPM,
                    target?.distanceCm ?: 0.0,
                    "teleop",
                )
            } else {
                0.0
            }
            if (flywheel.anyMotor) {
                when {
                    manualFire || assistFire -> flywheel.spinToRpm(shootingRpm)
                    FLYWHEEL_PRIME_ENABLED -> flywheel.prime()
                    else -> flywheel.stop()
                }
            }
            val readyToFire = flywheel.anyMotor && flywheel.isAtSpeed(shootingRpm)

            // ---- intake: manual only — the LOADER meters balls during fire ----
            if (hw.intakeMotor) {
                when {
                    gamepad1.a -> intake.spinUp(1.0)
                    gamepad1.b -> intake.spinUp(-1.0)
                    else -> intake.stop()
                }
            }

            // ---- loader: hold X = fire balls one per stroke (manual fire);
            // the align assist strokes automatically once BOTH wheels are at
            // the commanded speed. Each stroke is confirmed by the RPM dip. ----
            val wantStroke = loader.available &&
                (manualFire || assistFire) && (gamepad1.x || assistFire) &&
                readyToFire && !loader.busy && loader.phase == LoaderPhase.HOME
            if (wantStroke) {
                loader.stroke()
                loadState = LoadState.STROKING
                BlackBox.event(
                    "fire_start", "rpm" to shootingRpm,
                    "source" to if (gamepad1.x) "manual" else "assist",
                    "hiveCm" to target?.distanceCm,
                )
            }
            when (loadState) {
                LoadState.STROKING ->
                    if (!loader.busy) {
                        shotDetector.arm()
                        loadState = LoadState.WATCHING
                    }
                LoadState.WATCHING ->
                    when (shotDetector.update()) {
                        ShotDetector.State.CONFIRMED, ShotDetector.State.MISSED -> {
                            if (loader.retract()) {
                                loadState = LoadState.RETRACTING
                            } else {
                                // loader absent (manual-only robot): skip retract
                                shotDetector.idle()
                                loadState = LoadState.IDLE
                            }
                        }
                        else -> {}
                    }
                LoadState.RETRACTING ->
                    if (!loader.busy && loader.phase == LoaderPhase.HOME) {
                        shotDetector.idle()
                        loadState = LoadState.IDLE
                    }
                LoadState.IDLE -> {}
            }

            // ---- BALL CAPTURE ASSIST (hold LEFT bumper; needs vision) ----
            if (gamepad1.left_bumper && vision != null && ballLocator != null) {
                val ball = ballLocator.nearestBall()
                if (ball != null) {
                    val strafeAssist = cfg.CAPTURE_BEARING_GAIN * ball.bearingDeg
                    val approachAssist = cfg.CAPTURE_RANGE_GAIN *
                        (ball.distanceCm - cfg.CAPTURE_APPROACH_CM)
                    strafe = (strafe + strafeAssist).coerceIn(-1.0, 1.0)
                    forward = (forward + approachAssist.coerceIn(-cfg.ASSIST_MAX_POWER, cfg.ASSIST_MAX_POWER))
                        .coerceIn(-1.0, 1.0)
                }
            }

            // ---- TILT GUARD: cut drive power, not just warn (the legality ----
            // review describes this reflex as "cuts power") ----
            val tipping = tiltGuard.isTipping()
            if (tipping) {
                forward = 0.0
                strafe = 0.0
                turn = 0.0
            }

            // ---- drive (always works — that's the point of the fallback) ----
            // ---- POSE HOLD: sticks released -> hold the last still pose ----
            if (locator != null && poseHold) {
                val idleInput = abs(forward) < TELEOP_INPUT_DEADZONE &&
                    abs(strafe) < TELEOP_INPUT_DEADZONE &&
                    abs(turn) < TELEOP_INPUT_DEADZONE
                if (idleInput) {
                    if (stillPose == null) stillPose = locator.pose
                    // Body-frame twist back to the still pose (+x fwd, +y left, +angle CCW)
                    val err = stillPose!!.minus(locator.pose)
                    val hf = (TELEOP_HOLD_POSITION_KP * err.line.x)
                        .coerceIn(-TELEOP_HOLD_MAX_POWER, TELEOP_HOLD_MAX_POWER)
                    val hs = (TELEOP_HOLD_POSITION_KP * err.line.y)
                        .coerceIn(-TELEOP_HOLD_MAX_POWER, TELEOP_HOLD_MAX_POWER)
                    val ht = (TELEOP_HOLD_HEADING_KP * Math.toDegrees(err.angle))
                        .coerceIn(-TELEOP_HOLD_MAX_POWER, TELEOP_HOLD_MAX_POWER)
                    forward = (forward + hf).coerceIn(-1.0, 1.0)
                    strafe = (strafe + hs).coerceIn(-1.0, 1.0)
                    turn = (turn + ht).coerceIn(-1.0, 1.0)
                } else {
                    stillPose = null
                }
            }

            drivetrain?.driveTeleOp(forward, strafe, turn)
            vision?.periodic()

            if (manualOnly) {
                telemetry.addLine("!! AUTONOMOUS HARDWARE NOT DETECTED !!")
                telemetry.addLine("-> MANUAL ONLY mode")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
            }
            hive?.addTelemetry(telemetry, target)

            // ---- Black Box + SHADOW PLANNER (observes, never acts) ----
            val now = System.nanoTime()
            if (vision != null && ballLocator != null &&
                now - lastShadowNanos > 1_000_000_000L
            ) {
                lastShadowNanos = now
                shadowWorld.update(ballLocator.sightings(), null, null, 0.0, now)
                shadowWorld.updateHive(target, null, null, 0.0, now)
                val remaining = (MATCH_TELEOP_SECONDS - (runtime - clockStart)).coerceAtLeast(0.0)
                // hypothetical: 1 ball held — reveals the AI's ball preference
                // and endgame discipline without knowing our real payload.
                val d = shadowPlanner.decide(shadowWorld, 1, remaining, 0.0, 0.0)
                shadowPlanner.logDecision(d, "shadow", 1, remaining)
            }
            if (now - lastLogNanos > (1e9 / BLACKBOX_STATE_LOG_HZ).toLong()) {
                lastLogNanos = now
                BlackBox.log(
                    "record" to "teleop",
                    "fwd" to forward, "strafe" to strafe, "turn" to turn,
                    "a" to gamepad1.a, "b" to gamepad1.b,
                    "rb" to gamepad1.right_bumper, "lb" to gamepad1.left_bumper,
                    "trigger" to gamepad1.right_trigger.toDouble(),
                    "hiveCm" to target?.distanceCm, "hiveBearing" to target?.bearingDeg,
                    "shootingRpm" to shootingRpm, "atSpeed" to readyToFire,
                    "flyL" to flywheel.measuredLeftRpm(),
                    "flyR" to flywheel.measuredRightRpm(),
                    "batteryV" to ShotPredictor.reportHardwareMapVolts(hardwareMap),
                    "tiltDeg" to tiltGuard.tiltDeg(),
                    "shots" to loader.strokeCount,
                )
                shadowWorld.logToBlackBox()
            }
            if (tipping) telemetry.addLine("!! TILT GUARD — drive power cut !!")
            telemetry.addData("LOADER", "%d strokes fired".format(loader.strokeCount))

            telemetry.addData(
                "FLYWHEEL",
                "cmd %.0f  L %.0f  R %.0f RPM  %s".format(
                    shootingRpm,
                    flywheel.measuredLeftRpm(),
                    flywheel.measuredRightRpm(),
                    if (readyToFire) "AT SPEED" else "spinning up",
                ),
            )
            val ball = if (vision != null) ballLocator?.nearestBall() else null
            if (ball != null) {
                telemetry.addData(
                    "BALL ${ball.label}",
                    "%.1f cm, %.1f deg".format(ball.distanceCm, ball.bearingDeg),
                )
            } else if (!manualOnly) {
                telemetry.addData("BALL", "none visible")
            }
            telemetry.addData(
                "Assists",
                if (manualOnly) "OFF (manual only)"
                else if (alignAssist) "SHOOT"
                else if (gamepad1.left_bumper) "CAPTURE" else "off",
            )
            if (locator != null) {
                telemetry.addData(
                    "Drive mode",
                    "%s%s | LStkBtn toggles | RStkBtn hold %s".format(
                        if (fieldCentric) "FIELD-CENTRIC" else "robot-centric",
                        if (poseHold && stillPose != null) " [HOLDING]" else "",
                        if (poseHold) "ON" else "OFF",
                    ),
                )
                telemetry.addData(
                    "Pose",
                    "(%.2f, %.2f) %.1f deg | loc: %s".format(
                        locator.pose.position.x, locator.pose.position.y,
                        Math.toDegrees(locator.pose.heading.toDouble()),
                        if (locator.usingDeadWheels) "dead wheels" else "drive motors",
                    ),
                )
            }
            vision?.addTelemetry(telemetry)
            telemetry.update()
        }

        // ---- stop cleanly ----
        drivetrain?.flushMotors()
        intake.stop()
        flywheel.stop()
        vision?.close()
        BlackBox.stop()
    }
}
