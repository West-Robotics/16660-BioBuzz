package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.blackbox.BlackBox
import org.firstinspires.ftc.teamcode.blackbox.FrameGrabProcessor
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

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
        val drivetrain = Drivetrain()
        val intake = Intake()
        val flywheel = FlyWheel()
        val loader = Loader().also { it.home() }
        val shotDetector = ShotDetector(flywheel)

        val aprilTagProcessor = AprilTagProcessor.Builder()
            .setDrawTagOutline(true)
            .setDrawAxes(false)
            .build()
        val tagOverlay = TagInfoOverlay()
        val ballLocator = BallLocator()
        val upFrames = FrameGrabProcessor("UP")
        val frontFrames = FrameGrabProcessor("FRONT")

        val vision = VisionSubsystem(
            hardwareMap,
            extraUpProcessors = listOf(aprilTagProcessor, tagOverlay, upFrames),
            extraFrontProcessors = ballLocator.processors + frontFrames,
        )
        val hive = HiveScoring(aprilTagProcessor)
        val cfg = HiveScoringConfig

        // ---- Black Box (full-match recorder) + SHADOW PLANNER ----
        // The shadow planner watches the human drive and logs what IT would
        // do each second — pure observation, it never touches the motors.
        BlackBox.start(hardwareMap.appContext, "teleop")
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
            telemetry.update()
        }

        clockStart = runtime
        BlackBox.event("match_start", "alliance" to SELECTED_TEAM.name)

        while (opModeIsActive()) {
            // ---- driver input ----
            var forward = (-gamepad1.left_stick_y).toDouble()
            var strafe = gamepad1.left_stick_x.toDouble()
            var turn = gamepad1.right_stick_x.toDouble()

            // ---- latest hive measurement (used by assist + flywheel + telemetry) ----
            val target = hive.update()
            tagOverlay.publish(target)

            // ---- SHOOTING ALIGN ASSIST (hold RIGHT bumper) ----
            val alignAssist = gamepad1.right_bumper
            if (alignAssist) {
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
            val assistFire = alignAssist && hive.zoneOf(target) == ScoringZone.ALIGNED
            val shootingRpm = hive.shootingRpmFor(target)
            when {
                manualFire || assistFire -> flywheel.spinToRpm(shootingRpm)
                FLYWHEEL_PRIME_ENABLED -> flywheel.prime()
                else -> flywheel.stop()
            }
            val readyToFire = flywheel.isAtSpeed(shootingRpm)

            // ---- intake: manual only — the LOADER meters balls during fire ----
            when {
                gamepad1.a -> intake.spinUp(1.0)
                gamepad1.b -> intake.spinUp(-1.0)
                else -> intake.stop()
            }

            // ---- loader: hold X = fire balls one per stroke (manual fire);
            // the align assist strokes automatically once BOTH wheels are at
            // the commanded speed. Each stroke is confirmed by the RPM dip. ----
            val wantStroke = (manualFire || assistFire) && (gamepad1.x || assistFire) &&
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
                        ShotDetector.State.CONFIRMED, ShotDetector.State.MISSED ->
                            if (loader.retract()) loadState = LoadState.RETRACTING
                        else -> {}
                    }
                LoadState.RETRACTING ->
                    if (!loader.busy && loader.phase == LoaderPhase.HOME) {
                        shotDetector.idle()
                        loadState = LoadState.IDLE
                    }
                LoadState.IDLE -> {}
            }

            // ---- BALL CAPTURE ASSIST (hold LEFT bumper) ----
            if (gamepad1.left_bumper) {
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

            // ---- drive (existing NextFTC drivetrain) + fusion + telemetry ----
            drivetrain.driveTeleOp(forward, strafe, turn)
            vision.periodic()

            hive.addTelemetry(telemetry, target)

            // ---- Black Box + SHADOW PLANNER (observes, never acts) ----
            val now = System.nanoTime()
            if (now - lastShadowNanos > 1_000_000_000L) {
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
                    "batteryV" to (try {
                        hardwareMap.voltageSensor.iterator().next().voltage
                    } catch (_: Exception) { 0.0 }),
                    "tiltDeg" to tiltGuard.tiltDeg(),
                    "shots" to loader.strokeCount,
                )
                shadowWorld.logToBlackBox()
            }
            if (tiltGuard.isTipping()) telemetry.addLine("!! TILT WARNING — ease off !!")
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
            val ball = ballLocator.nearestBall()
            if (ball != null) {
                telemetry.addData(
                    "BALL ${ball.label}",
                    "%.1f cm, %.1f deg".format(ball.distanceCm, ball.bearingDeg),
                )
            } else {
                telemetry.addData("BALL", "none visible")
            }
            telemetry.addData("Assists", if (alignAssist) "SHOOT" else if (gamepad1.left_bumper) "CAPTURE" else "off")
            vision.addTelemetry(telemetry)
            telemetry.update()
        }

        // ---- stop cleanly ----
        drivetrain.flushMotors()
        intake.stop()
        flywheel.stop()
        vision.close()
        BlackBox.stop()
    }
}
