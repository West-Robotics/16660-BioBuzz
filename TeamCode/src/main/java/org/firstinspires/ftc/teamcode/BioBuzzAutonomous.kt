package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.pedropathing.ivy.behaviors.EndCondition
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import org.firstinspires.ftc.teamcode.vision.VisionTuning
import org.firstinspires.ftc.teamcode.vision.visionGuidance
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

/**
 * Field-specific route plan. The collection phase is now vision-directed
 * (see globals.kt priority + collection constants); only the park move
 * remains a fixed trajectory. TUNE for the actual venue.
 */
object AutoPlan {
    const val PARK_BACKOFF_IN = 12.0 // TUNE: back-off after firing
}

/**
 * ============ BIOBUZZ AUTONOMOUS — the closed ecosystem ============
 *
 * Phases (each timeout-bounded, safe-stopped, telemetry-visible):
 *   1. COLLECT: intake on, RR trajectory sweep through the collection area.
 *   2. POLISH:  vision-guidance command nudges the pose against stationary
 *              visual references (no-ops unless vision is confident).
 *   3. ALIGN:   search for the hive tag, servo into the shooting window;
 *              flywheel pre-spins to the RPM the live distance needs.
 *   4. FIRE:    spin to the calibrated RPM, wait until BOTH wheels are at
 *              speed, then feed balls through for FIRE_TIME_MS.
 *   5. PARK:    back off from the hive and stop.
 *
 * Prereqs on the real robot: RR tuning (MecanumDrive.PARAMS), "imu" configured,
 * vision test procedure passed, [HiveScoringConfig] measured.
 *
 * ALLIANCE: pick RED or BLUE on the gamepad during INIT (D-pad LEFT/RIGHT);
 * only the selected alliance's hive tags are ever targeted (SELECTED_TEAM).
 */
@Autonomous(name = "BioBuzz Autonomous", group = "Auto")
class BioBuzzAutonomous : LinearOpMode() {

    private lateinit var drive: MecanumDrive
    private lateinit var vision: VisionSubsystem
    private lateinit var hive: HiveScoring
    private lateinit var tagOverlay: TagInfoOverlay
    private lateinit var ballLocator: BallLocator
    private lateinit var intake: Intake
    private lateinit var flywheel: FlyWheel
    private lateinit var loader: Loader
    private val world = WorldModel()

    override fun runOpMode() {
        // ---- boot-time hardware probe: no full autonomy kit -> stay parked ----
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.fullAutonomy) {
            BlackBox.start(hardwareMap.appContext, "auto")
            hw.logToBlackBox()
            BlackBox.event("autonomy_unavailable")
            while (!isStopRequested) {
                telemetry.addLine("!! AUTONOMOUS HARDWARE NOT DETECTED !!")
                telemetry.addLine("Autonomous stays PARKED — run a TeleOp instead.")
                telemetry.addData("Missing", hw.missingList.joinToString(", "))
                telemetry.update()
                sleep(50) // don't busy-spin a core during INIT
            }
            BlackBox.stop()
            return
        }

        // ================= initialize =================
        drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))

        // Known-target sensors share the localization portals (AprilTag on the
        // UP camera, ball blobs on the FRONT camera — no extra cameras).
        val aprilTagProcessor = AprilTagProcessor.Builder()
            .setDrawTagOutline(true)
            .setDrawAxes(false)
            .setDrawCubeProjection(false)
            .build()
        tagOverlay = TagInfoOverlay()
        ballLocator = BallLocator()

        vision = VisionSubsystem(
            hardwareMap,
            extraUpProcessors = listOf(aprilTagProcessor, tagOverlay, FrameGrabProcessor("UP")),
            extraFrontProcessors = ballLocator.processors + FrameGrabProcessor("FRONT") +
                DetectorProcessor(hardwareMap.appContext),
        )
        hive = HiveScoring(aprilTagProcessor)
        intake = Intake()
        flywheel = FlyWheel()
        loader = Loader().also { it.home() }

        vision.attachTo(drive, enableCorrections = VisionTuning.FUSION_ENABLED_DEFAULT)
        BlackBox.start(hardwareMap.appContext, "auto")
        hw.logToBlackBox()
        ShotPredictor.ensureLoaded(hardwareMap.appContext)

        // ---- INIT: pick your alliance before the match starts ----
        // D-pad LEFT = RED, D-pad RIGHT = BLUE. HiveScoring only ever targets
        // the selected alliance's hives, so this must match the match sheet.
        while (!isStarted && !isStopRequested) {
            if (gamepad1.dpad_left) SELECTED_TEAM = TeamColor.RED
            else if (gamepad1.dpad_right) SELECTED_TEAM = TeamColor.BLUE

            telemetry.addLine("BioBuzz autonomous: RR + hybrid vision + AprilTag scoring")
            telemetry.addData(
                "Vision corrections",
                if (drive.visionCorrectionEnabled) "ENABLED" else "disabled (pass vision tests first)",
            )
            telemetry.addData(
                "Shoot range",
                "${HiveScoringConfig.SHOOT_MIN_RANGE_CM.toInt()}-" +
                    "${ShooterModel.effectiveMaxRangeCm().toInt()} cm " +
                    "(${ShooterModel.summary()}), " +
                    "|angle| <= ${HiveScoringConfig.SHOOT_MAX_BEARING_DEG.toInt()} deg",
            )
            telemetry.addData("ALLIANCE", SELECTED_TEAM.name)
            telemetry.addLine("Select with D-pad: LEFT = RED, RIGHT = BLUE")
            if (MecanumDrive.PARAMS.kV == 0.0 || MecanumDrive.PARAMS.axialGain == 0.0) {
                telemetry.addLine("!! RR UNTUNED (MecanumDrive.PARAMS are zeros) !!")
                telemetry.addLine("-> trajectory moves (e.g. PHASE 5 park) will NOT drive the robot")
            }
            telemetry.update()
        }

        try {
            // ---- PHASE 1: COLLECT (vision-directed, capacity-aware) ----
            telemetry.addLine("PHASE 1: vision-directed collection")
            BlackBox.event("phase", "name" to "1-collect")
            val collected = collectBalls()
            BlackBox.event("collect_done", "balls" to collected)
            telemetry.addData("PHASE 1 done", "$collected / $MAX_BALLS_HELD balls")

            // ---- PHASE 1b: FLOWER (optional, customizable pull-out) ----
            if (FLOWER_PULL_ENABLED && collected < MAX_BALLS_HELD) {
                telemetry.addLine("PHASE 1b: flower pull")
                BlackBox.event("phase", "name" to "1b-flower")
                useFlowerIfVisible()
            }

            // ---- PHASE 2: POLISH (vision-guided pose alignment) ----
            telemetry.addLine("PHASE 2: vision-guidance polish")
            BlackBox.event("phase", "name" to "2-polish")
            runGuidance()

            // ---- PHASE 3: ALIGN (AprilTag window; flywheel pre-spin) ----
            // Pre-spin targets the RPM the CURRENT hive distance needs (the
            // calibrated curve), so the wheels are already at shooting speed
            // by the time alignment finishes; no tag yet = hold the prime speed.
            telemetry.addLine("PHASE 3: hive alignment")
            BlackBox.event("phase", "name" to "3-align")
            flywheel.spinToRpm(
                hive.update()?.let { ShooterModel.rpmForDistance(it.distanceCm) }
                    ?: FLYWHEEL_PRIME_RPM
            )
            val aligned = alignToHive()

            // ---- PHASE 4: FIRE (at the calibrated speed, feed until done) ----
            BlackBox.event("phase", "name" to "4-fire")
            if (aligned) {
                val fireTarget = hive.update()
                val rpm = ShotPredictor.adjustRpm(
                    hive.shootingRpmFor(fireTarget),
                    fireTarget?.distanceCm ?: 0.0,
                    "auto",
                )
                flywheel.spinToRpm(rpm)
                // "hiveCm" is required by tools/shooter_refit.py — without it
                // every autonomous shot is silently unusable for curve fitting.
                BlackBox.event("fire_start", "rpm" to rpm, "hiveCm" to fireTarget?.distanceCm)

                // Wait until BOTH wheels are actually at the commanded speed
                // (the 6000 RPM motors spin up slowly) before feeding balls.
                val spinDeadline = System.nanoTime() +
                    (HiveScoringConfig.FIRE_TIME_MS * 1e6).toLong()
                while (opModeIsActive() && !flywheel.isAtSpeed(rpm) &&
                    System.nanoTime() < spinDeadline
                ) {
                    telemetry.addData("PHASE 4", "spinning up to %.0f RPM".format(rpm))
                    telemetry.addData(
                        "Measured L/R",
                        "%.0f / %.0f RPM".format(
                            flywheel.measuredLeftRpm(),
                            flywheel.measuredRightRpm(),
                        ),
                    )
                    telemetry.update()
                }

                if (!flywheel.isAtSpeed(rpm)) {
                    // Never feed balls into slow wheels: that's a weak shot
                    // (and a possible stuck ball). Safe-skip instead.
                    BlackBox.event("fire_aborted", "rpm" to rpm, "reason" to "wheels not at speed")
                    telemetry.addData("PHASE 4", "SKIP FIRE — wheels never reached %.0f RPM".format(rpm))
                    telemetry.update()
                } else {
                // Fire every held ball: one LOADER stroke per ball, CONFIRMED
                // by the flywheel RPM dip, wheels re-verified at speed between
                // balls. The intake does NOT run during fire — the loader
                // meters balls straight from the hopper.
                telemetry.addData("PHASE 4", "FIRING at %.0f RPM".format(rpm))
                telemetry.update()
                val detector = ShotDetector(flywheel)
                var fired = 0
                var stroking = false
                var armed = false
                while (fired < collected && opModeIsActive()) {
                    when {
                        stroking && loader.busy -> report()
                        stroking && !armed -> {
                            detector.arm()
                            armed = true
                        }
                        armed -> when (detector.update()) {
                            ShotDetector.State.CONFIRMED, ShotDetector.State.MISSED -> {
                                fired++
                                stroking = false
                                armed = false
                                loader.retract()
                                // let the wheels recover before the next ball
                                val recoverBy = System.nanoTime() +
                                    (SHOT_RECOVERY_TIMEOUT_MS * 1e6).toLong()
                                while (opModeIsActive() && !flywheel.isAtSpeed(rpm) &&
                                    System.nanoTime() < recoverBy
                                ) {
                                    report()
                                }
                            }
                            else -> report()
                        }
                        !loader.busy && loader.phase == LoaderPhase.HOME -> {
                            loader.stroke()
                            stroking = true
                        }
                        !loader.busy -> loader.retract() // settle any mid-cycle state
                        else -> report()
                    }
                }
                BlackBox.event("fire_done", "rpm" to rpm, "fired" to fired, "attempted" to collected)
                }
            } else {
                telemetry.addData("PHASE 4", "ALIGN FAILED - skipping fire (safe)")
                telemetry.update()
            }
            flywheel.stop()

            // ---- PHASE 5: PARK ----
            telemetry.addLine("PHASE 5: park")
            BlackBox.event("phase", "name" to "5-park")
            runAction(
                drive.actionBuilder(drive.pose)
                    .lineToXConstantHeading(drive.pose.position.x - AutoPlan.PARK_BACKOFF_IN)
                    .build(),
            )
        } finally {
            // Always leave the robot safe, no matter where the phases exit.
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            intake.stop()
            flywheel.stop()
            vision.close()
            BlackBox.event("auto_end")
            BlackBox.stop()
        }
    }

    // ================= phase helper loops =================

    /**
     * Runs a Road Runner action while continuously feeding the hybrid vision
     * fusion + telemetry. Stops the drivetrain immediately if the OpMode ends
     * mid-trajectory (action.run() does not know about opModeIsActive).
     */
    private fun runAction(action: Action) {
        val packet = TelemetryPacket()
        var running = true
        while (running && opModeIsActive()) {
            running = action.run(packet) // also updates pose (and vision correction)
            vision.updateRoadRunnerPose(drive.pose)
            report()
        }
        if (!opModeIsActive()) {
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        }
    }

    /**
     * Inline-executes the NextFTC vision-guidance command. It only moves the
     * robot while vision confidence is high; the command's own timeout ends it.
     */
    private fun runGuidance() {
        val cmd = visionGuidance(drive, vision)
        cmd.start()
        while (!cmd.done() && opModeIsActive()) {
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)
            cmd.execute()
            report()
        }
        cmd.end(if (cmd.done()) EndCondition.NATURALLY else EndCondition.INTERRUPTED)
    }

    /**
     * PHASE 1 core: collection driven by this branch's own decision stack.
     * Each loop:
     *   1. report() fuses this frame's sightings into the WorldModel,
     *   2. the StrategicPlanner picks the action. COLLECT_BALL commits to a
     *      remembered TRACK — a challenger must win TARGET_SWITCH_CONFIRM_LOOPS
     *      consecutive picks to unseat it, which kills target dithering;
     *      GO_SCORE / USE_FLOWER / PARK exit the phase early so the phase
     *      machine can act on the strategy immediately,
     *   3. dodge opponent robots sitting in the pursuit corridor,
     *   4. pursue the track's FIELD position — dead-reckoned on the fused
     *      pose when the ball has left the FOV — with a turn-toward-ball
     *      term so wide bearings rotate the intake onto the ball,
     *   5. confirm pickup by proximity-settle, then removeBall() so the
     *      map never re-chases a collected ball,
     *   6. nothing worth doing (HOLD): sweep-search in place instead of the
     *      old blind forward creep.
     * Bounded by AUTONOMOUS_COLLECT_TIMEOUT_MS.
     */
    private fun collectBalls(): Int {
        var collected = 0
        var committedId: Int? = null
        var challengerId: Int? = null
        var challengerWins = 0
        var closeSinceNanos = 0L
        val planner = StrategicPlanner()
        val startNanos = System.nanoTime()
        val deadline = startNanos + (AUTONOMOUS_COLLECT_TIMEOUT_MS * 1e6).toLong()

        intake.spinUp(1.0)
        while (opModeIsActive() && collected < MAX_BALLS_HELD && System.nanoTime() < deadline) {
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)
            report() // fuses sightings + hive into the WorldModel (and Black Box)

            val now = System.nanoTime()
            val px = drive.pose.position.x * 2.54
            val py = drive.pose.position.y * 2.54
            val hdg = drive.pose.heading.toDouble()
            // The planner reasons in MATCH time, not collect-window time.
            val matchRemainingSec = AUTONOMY_MATCH_BUDGET_SEC - (now - startNanos) / 1e9

            // ---- 2. planner decision ----
            val decision = planner.decide(world, collected, matchRemainingSec, px, py)
            when (decision.action) {
                PlannerAction.GO_SCORE, PlannerAction.USE_FLOWER, PlannerAction.PARK -> {
                    // Leave collection to the phase machine (flower / align / fire / park).
                    BlackBox.event(
                        "collect_exit",
                        "action" to decision.action.name,
                        "balls" to collected,
                        "why" to decision.rationale,
                    )
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                    intake.stop()
                    return collected
                }
                else -> {}
            }

            // ---- commitment with confirmation hysteresis ----
            if (decision.action == PlannerAction.COLLECT_BALL) {
                val current = committedId?.let { world.trackById(it) }
                if (current == null) {
                    committedId = decision.targetId
                    challengerId = null; challengerWins = 0
                } else if (decision.targetId != committedId) {
                    if (challengerId == decision.targetId) challengerWins++ else challengerWins = 1
                    challengerId = decision.targetId
                    if (challengerWins >= TARGET_SWITCH_CONFIRM_LOOPS) {
                        BlackBox.event(
                            "target_switch",
                            "from" to committedId, "to" to decision.targetId, "why" to decision.rationale,
                        )
                        committedId = decision.targetId
                        challengerId = null; challengerWins = 0
                    }
                } else {
                    challengerId = null; challengerWins = 0
                }
            }

            var target = committedId?.let { world.trackById(it) }
            // Chase eligibility: fresh-enough sighting of a solid-enough blob.
            if (target != null &&
                (target.ageSec(now) > DEAD_RECKON_MAX_AGE_SEC || target.lastAreaPx < CHASE_MIN_CONTOUR_AREA_PX)
            ) {
                committedId = null
                target = null
            }

            if (target == null) {
                // ---- 6. nothing worth chasing: sweep-search in place ----
                closeSinceNanos = 0L
                telemetry.addData("COLLECT", "no target — sweep | ${world.summary()}")
                telemetry.update()
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), -SEARCH_SWEEP_TURN_POWER))
                continue
            }

            val (distCm, bearingDeg) = target.relativeToRobot(px, py, hdg)

            // ---- 3. opponent dodge: robot blob in the pursuit corridor ----
            val blocker = ballLocator.visibleRobots().firstOrNull {
                abs(it.bearingDeg - bearingDeg) < AVOID_ROBOT_BEARING_DEG && it.distanceCm < distCm
            }
            if (blocker != null) {
                // Drop the commitment so next frame re-approaches from a new angle.
                committedId = null
                closeSinceNanos = 0L
                val dodge = if (blocker.bearingDeg >= 0) AVOID_ROBOT_STRAFE else -AVOID_ROBOT_STRAFE
                telemetry.addData("COLLECT dodge", "robot %.0fcm %+.0fdeg".format(blocker.distanceCm, blocker.bearingDeg))
                telemetry.update()
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, dodge), 0.0))
                continue
            }

            // ---- 4. pursue the track's field position (live or remembered) ----
            val forward = (HiveScoringConfig.CAPTURE_RANGE_GAIN *
                (distCm - HiveScoringConfig.CAPTURE_APPROACH_CM))
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val strafe = (-HiveScoringConfig.CAPTURE_BEARING_GAIN * bearingDeg)
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            // Turn toward the ball while translating (RR: +turn CCW).
            val turn = (-CAPTURE_TURN_GAIN * bearingDeg)
                .coerceIn(-CAPTURE_TURN_MAX_POWER, CAPTURE_TURN_MAX_POWER)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, strafe), turn))

            val live = target.ageSec(now) <= WM_VISIBLE_FRESH_SEC
            telemetry.addData(
                "COLLECT $collected/$MAX_BALLS_HELD",
                "%s %.0fcm %+.1fdeg %s".format(
                    target.label, distCm, bearingDeg, if (live) "[live]" else "[memory]",
                ),
            )
            telemetry.update()

            // ---- 5. pickup confirmation: close for long enough ----
            if (distCm <= PICKUP_CONFIRM_CM) {
                if (closeSinceNanos == 0L) {
                    closeSinceNanos = now
                } else if ((now - closeSinceNanos) / 1e6 >= PICKUP_SETTLE_MS) {
                    collected++
                    world.removeBall(target.id)
                    committedId = null
                    closeSinceNanos = 0L
                    telemetry.addData("COLLECT pickup", "${target.label} (settled)")
                    telemetry.update()
                }
            } else {
                closeSinceNanos = 0L
            }
        }

        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        intake.stop()
        return collected
    }

    /**
     * PHASE 1b: optional flower use. Drives to FLOWER_APPROACH_CM of the
     * nearest visible flower, stops, and runs the CUSTOMIZABLE
     * flowerPullRoutine from globals.kt — edit that lambda to match the real
     * mechanism. Skips silently when no flower is visible or the approach
     * times out. Returns true when a pull was actually performed.
     */
    private fun useFlowerIfVisible(): Boolean {
        val deadline = System.nanoTime() + (FLOWER_PULL_TIMEOUT_MS * 1e6).toLong()
        var positioned = false

        while (opModeIsActive() && System.nanoTime() < deadline) {
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)

            val flower = ballLocator.nearestFlower()
            report()
            if (flower == null) break // nothing visible: skip the phase
            telemetry.addData(
                "FLOWER",
                "%.0fcm %+.1fdeg".format(flower.distanceCm, flower.bearingDeg),
            )
            telemetry.update()

            if (flower.distanceCm <= FLOWER_APPROACH_CM + 5.0) {
                positioned = true
                break
            }

            val forward = (HiveScoringConfig.CAPTURE_RANGE_GAIN *
                (flower.distanceCm - FLOWER_APPROACH_CM))
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val strafe = (-HiveScoringConfig.CAPTURE_BEARING_GAIN * flower.bearingDeg)
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, strafe), 0.0))
        }

        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        if (!positioned) return false

        // ==== CUSTOMIZABLE PULL-OUT — edit flowerPullRoutine in globals.kt ====
        flowerPullRoutine(intake, loader)

        // Give the released ball time to settle into the intake.
        intake.spinUp(1.0)
        sleep(FLOWER_SETTLE_AFTER_PULL_MS)
        intake.stop()
        return true
    }

    /**
     * PHASE 3 core: sweep for the hive tag, then servo into the shooting
     * window (turn to center the tag, drive to range-center, stop when
     * ALIGNED). Bounded by ALIGN_TIMEOUT_MS. Returns true only when the robot
     * actually reached ALIGNED — firing is skipped otherwise.
     */
    private fun alignToHive(): Boolean {
        val deadline = System.nanoTime() + (HiveScoringConfig.ALIGN_TIMEOUT_MS * 1e6).toLong()
        while (opModeIsActive() && System.nanoTime() < deadline) {
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)

            val target = hive.update()
            tagOverlay.publish(target)
            report(target)

            if (target == null) {
                // Sweep in place until a hive tag appears.
                drive.setDrivePowers(
                    PoseVelocity2d(Vector2d(0.0, 0.0), -HiveScoringConfig.SEARCH_TURN_POWER),
                )
                continue
            }

            if (hive.zoneOf(target) == ScoringZone.ALIGNED) {
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                return true
            }

            val (forward, turn) = hive.rangeCorrection(target)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, 0.0), turn))
        }
        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        return false
    }

    /** Combined telemetry: hive range/angle, RR (fused) pose, vision health. */
    private fun report(target: HiveTarget? = null) {
        val t = target ?: hive.update()
        tagOverlay.publish(t)
        hive.addTelemetry(telemetry, t)

        val p = drive.pose
        val px = p.position.x * 2.54
        val py = p.position.y * 2.54
        val hdg = p.heading.toDouble()
        val now = System.nanoTime()
        world.update(ballLocator.sightings(), px, py, hdg, now)
        world.updateHive(t, px, py, hdg, now)
        BlackBox.log(
            "record" to "auto",
            "poseXcm" to px, "poseYcm" to py, "headingRad" to hdg,
            "hiveCm" to t?.distanceCm, "hiveBearing" to t?.bearingDeg,
            "flyL" to flywheel.measuredLeftRpm(),
            "flyR" to flywheel.measuredRightRpm(),
            "batteryV" to ShotPredictor.reportHardwareMapVolts(hardwareMap),
        )
        world.logToBlackBox()

        telemetry.addData(
            "RR/FUSED pose",
            "(%.2f, %.2f) %.1f deg".format(p.position.x, p.position.y, Math.toDegrees(hdg)),
        )
        vision.addTelemetry(telemetry)
        telemetry.update()
    }
}
