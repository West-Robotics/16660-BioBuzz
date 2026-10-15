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
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

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
 * A time-budgeted SCORE LOOP (see [PhaseClock]) instead of a fixed phase
 * list: while a full collect+align+fire cycle fits in the remaining
 * autonomous period, keep cycling —
 *   1. COLLECT: intake on, memory-driven pursuit of remembered balls
 *      (see collectBalls; the BallMemory persists across cycles, so balls
 *      glimpsed while scoring are already mapped for the next cycle).
 *   1b. FLOWER: optional pull-out when capacity remains.
 *   2. POLISH: vision-guidance pose cleanup (first cycle only).
 *   3. ALIGN: AprilTag window; dead-reckons toward the remembered hive
 *      position when the tag flickers out of view.
 *   4. FIRE: per-ball feeding — spin-up, one feed pulse per ball,
 *      wheels re-verified at speed between balls.
 *   5. PARK: always runs — straight back along the robot's own heading
 *      (the park time is reserved by the clock, so nothing else eats it).
 *
 * Prereqs on the real robot: RR tuning (MecanumDrive.PARAMS), "imu" configured,
 * vision test procedure passed, [HiveScoringConfig] measured. With the
 * odometry pods configured ("par0"/"par1"/"perp"), localization runs on dead
 * wheels; otherwise it falls back to drive-motor odometry.
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
    private lateinit var clock: PhaseClock
    private lateinit var pickupSensor: PickupCurrentSensor

    /** Field memory of balls/flowers — PERSISTS across score cycles. */
    private val ballMemory = BallMemory()

    /** Remembered hive position (field cm) for tag-flicker dead reckoning. */
    private var hiveMemXcm = 0.0
    private var hiveMemYcm = 0.0
    private var hiveMemSeenNanos = 0L

    override fun runOpMode() {
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
            extraUpProcessors = listOf(aprilTagProcessor, tagOverlay),
            extraFrontProcessors = ballLocator.processors,
        )
        hive = HiveScoring(aprilTagProcessor)
        intake = Intake()
        flywheel = FlyWheel()

        vision.attachTo(drive, enableCorrections = VisionTuning.FUSION_ENABLED_DEFAULT)

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
            telemetry.addData(
                "Localization",
                if (drive.usingDeadWheels) "DEAD WHEELS (pods)" else "drive motors (add pods: par0/par1/perp)",
            )
            if (MecanumDrive.PARAMS.kV == 0.0 || MecanumDrive.PARAMS.axialGain == 0.0) {
                telemetry.addLine("!! RR UNTUNED (MecanumDrive.PARAMS are zeros) !!")
                telemetry.addLine("-> trajectory moves (e.g. PHASE 5 park) will NOT drive the robot")
            }
            telemetry.update()
        }

        clock = PhaseClock()
        pickupSensor = PickupCurrentSensor(intake)
        ballMemory.clear()

        try {
            // ---- THE SCORE LOOP: keep cycling while a full cycle fits ----
            var cycle = 0
            var totalScored = 0
            while (opModeIsActive()) {
                cycle++
                telemetry.addData("AUTO CYCLE", cycle)
                telemetry.addData("Clock", "%.1fs left".format(clock.remainingSec()))
                telemetry.update()

                // ---- PHASE 1: COLLECT (vision-directed, capacity-aware) ----
                val collectDeadline = clock.deadlineNanos(
                    AUTONOMOUS_COLLECT_TIMEOUT_MS,
                    reserveSec = AUTONOMY_PARK_RESERVE_SEC + AUTONOMY_FIRE_RESERVE_SEC,
                )
                val collected = collectBalls(collectDeadline)
                telemetry.addData("PHASE 1 done", "$collected / $MAX_BALLS_HELD balls")
                telemetry.update()

                // ---- PHASE 1b: FLOWER (optional; needs capacity AND time) ----
                if (FLOWER_PULL_ENABLED && collected < MAX_BALLS_HELD &&
                    clock.fits(MIN_CYCLE_REMAINING_SEC + AUTONOMY_FIRE_RESERVE_SEC)
                ) {
                    telemetry.addLine("PHASE 1b: flower pull")
                    useFlowerIfVisible(
                        clock.deadlineNanos(
                            FLOWER_PULL_TIMEOUT_MS,
                            reserveSec = AUTONOMY_PARK_RESERVE_SEC + AUTONOMY_FIRE_RESERVE_SEC,
                        )
                    )
                }

                // ---- PHASE 2: POLISH (first cycle only — one-time cleanup) ----
                if (cycle == 1) {
                    telemetry.addLine("PHASE 2: vision-guidance polish")
                    runGuidance()
                }

                // ---- PHASE 3 + 4: ALIGN + FIRE (only with balls to shoot) ----
                if (collected > 0) {
                    telemetry.addLine("PHASE 3: hive alignment")
                    flywheel.spinToRpm(
                        hive.update()?.let { ShooterModel.rpmForDistance(it.distanceCm) }
                            ?: FLYWHEEL_PRIME_RPM
                    )
                    val aligned = alignToHive(
                        clock.deadlineNanos(
                            HiveScoringConfig.ALIGN_TIMEOUT_MS,
                            reserveSec = AUTONOMY_PARK_RESERVE_SEC + AUTONOMY_FIRE_RESERVE_SEC,
                        )
                    )

                    if (aligned) {
                        val fired = fireBalls(collected)
                        totalScored += fired
                        telemetry.addData("PHASE 4", "$fired balls fired ($totalScored total)")
                        telemetry.update()
                    } else {
                        telemetry.addData("PHASE 4", "ALIGN FAILED - skipping fire (safe)")
                        telemetry.update()
                    }
                    flywheel.stop()
                } else if (!clock.fits(MIN_CYCLE_REMAINING_SEC)) {
                    // Nothing collected and no time for another try: park.
                    break
                }

                if (!clock.fits(MIN_CYCLE_REMAINING_SEC)) break
            }

            // ---- PHASE 5: PARK (time reserved by the clock; always runs) ----
            telemetry.addData("PHASE 5", "park (%.0fs left)".format(clock.remainingSec()))
            telemetry.update()
            // Back off ALONG THE ROBOT'S OWN HEADING (the old -X world move
            // was sideways/toward-hive whenever align had rotated the bot).
            val hdg = drive.pose.heading.toDouble()
            runAction(
                drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(
                        Vector2d(
                            drive.pose.position.x - AutoPlan.PARK_BACKOFF_IN * cos(hdg),
                            drive.pose.position.y - AutoPlan.PARK_BACKOFF_IN * sin(hdg),
                        ),
                    )
                    .build(),
            )
        } finally {
            // Always leave the robot safe, no matter where the phases exit.
            drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            intake.stop()
            flywheel.stop()
            vision.close()
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
     * PHASE 1 core: capacity-aware, vision-directed collection with FIELD
     * MEMORY (BallMemory — shared across score cycles). Each loop:
     *   1. fuse this frame's ball sightings into field coordinates,
     *   2. COMMIT to one remembered ball — a challenger must win by
     *      TARGET_SWITCH_MARGIN utility, which kills target dithering,
     *   3. dodge opponent robots in the pursuit corridor (direction latched
     *      for DODGE_LATCH_MS so a flickering blob can't oscillate the dodge;
     *      the target is only given up after BLOCKED_GIVEUP_MS),
     *   4. pursue with the SAME gains whether the ball is in view (visual
     *      servo) or only remembered (dead-reckoned on the fused pose), and
     *      TURN toward it while translating — converges wide bearings much
     *      faster than strafe alone and keeps the ball centered in the FOV,
     *   5. confirm pickup by the intake CURRENT bump (physical signature)
     *      or by proximity-settle, then forget the ball,
     *   6. nothing visible AND nothing remembered: sweep-search in place
     *      instead of the old blind forward creep.
     * Bounded by [deadlineNanos] (clock-clamped, never past the reserves).
     */
    private fun collectBalls(deadlineNanos: Long): Int {
        var collected = 0
        var committedId: Int? = null
        var closeSinceNanos = 0L
        var dodgeSign = 0        // latched dodge direction (+left / -right in RR strafe terms)
        var dodgeUntilNanos = 0L
        var blockedSinceNanos = 0L

        intake.spinUp(1.0)
        while (opModeIsActive() && collected < MAX_BALLS_HELD && System.nanoTime() < deadlineNanos) {
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)
            val now = System.nanoTime()
            val robotXcm = drive.pose.position.x * 2.54
            val robotYcm = drive.pose.position.y * 2.54

            // 1. fuse sightings into the field-frame ball map
            ballMemory.absorb(ballLocator.visibleTargets(), drive.pose, now)
            val robots = ballLocator.visibleRobots()
            val bumped = pickupSensor.update(spinning = true)
            report()

            // 2. commitment with switch hysteresis
            val committed = committedId?.let { ballMemory.byId(it) }
            val best = ballMemory.best(robotXcm, robotYcm, MAX_BALLS_HELD - collected, now)
            if (committed == null) {
                committedId = best?.id
            } else if (best != null && best.id != committed.id &&
                ballMemory.utilityOf(best, robotXcm, robotYcm) >
                ballMemory.utilityOf(committed, robotXcm, robotYcm) * TARGET_SWITCH_MARGIN
            ) {
                committedId = best.id
            }
            val target = committedId?.let { ballMemory.byId(it) }

            if (target == null) {
                // 6. nothing worth chasing: sweep-search in place
                closeSinceNanos = 0L
                dodgeSign = 0
                telemetry.addData("COLLECT", "no target — sweep | map: ${ballMemory.summary(now)}")
                telemetry.update()
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), -SEARCH_SWEEP_TURN_POWER))
                continue
            }

            val (distCm, bearingDeg) = ballMemory.relativeTo(target, drive.pose)

            // 3. opponent dodge: a robot blob in the pursuit corridor blocks the line
            val blocker = robots.firstOrNull {
                abs(it.bearingDeg - bearingDeg) < AVOID_ROBOT_BEARING_DEG && it.distanceCm < distCm
            }
            if (blocker != null) {
                // Latch the dodge direction once; only give the target up
                // (and re-approach from a new angle) after a long block.
                if (dodgeSign == 0) {
                    dodgeSign = if (blocker.bearingDeg >= 0) 1 else -1
                    dodgeUntilNanos = now + (DODGE_LATCH_MS * 1e6).toLong()
                }
                if (blockedSinceNanos == 0L) blockedSinceNanos = now
                closeSinceNanos = 0L

                val givingUp = (now - blockedSinceNanos) / 1e6 >= BLOCKED_GIVEUP_MS
                val stillLatched = now < dodgeUntilNanos
                if (givingUp || !stillLatched) {
                    dodgeSign = 0
                    blockedSinceNanos = 0L
                }
                if (givingUp) {
                    committedId = null // re-plan around the blocker
                    telemetry.addData("COLLECT dodge", "blocked too long — re-target")
                } else {
                    telemetry.addData(
                        "COLLECT dodge",
                        "robot %.0fcm %+.0fdeg".format(blocker.distanceCm, blocker.bearingDeg),
                    )
                }
                telemetry.update()
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, dodgeSign * AVOID_ROBOT_STRAFE), 0.0))
                continue
            } else {
                dodgeSign = 0
                blockedSinceNanos = 0L
            }

            // 4. pursue (live sighting or remembered spot — identical controller)
            val forward = (HiveScoringConfig.CAPTURE_RANGE_GAIN *
                (distCm - HiveScoringConfig.CAPTURE_APPROACH_CM))
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val strafe = (-HiveScoringConfig.CAPTURE_BEARING_GAIN * bearingDeg)
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            // Turn toward the ball while translating (RR: +turn CCW).
            val turn = (-CAPTURE_TURN_GAIN * bearingDeg)
                .coerceIn(-CAPTURE_TURN_MAX_POWER, CAPTURE_TURN_MAX_POWER)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, strafe), turn))

            telemetry.addData(
                "COLLECT $collected/$MAX_BALLS_HELD",
                "%s %.0fcm %+.1fdeg %s".format(
                    target.label, distCm, bearingDeg,
                    if (target.visibleNow) "[live]" else "[memory]",
                ),
            )
            telemetry.update()

            // 5. pickup confirmation. The current bump is the PHYSICAL
            //    signature (a ball just entered the intake); settle is the
            //    fallback when the bump is not detectable.
            val bumpPickup = bumped && distCm <= PICKUP_CONFIRM_CM * 1.5
            val settled = if (distCm <= PICKUP_CONFIRM_CM) {
                if (closeSinceNanos == 0L) {
                    closeSinceNanos = now
                    false
                } else {
                    (now - closeSinceNanos) / 1e6 >= PICKUP_SETTLE_MS
                }
            } else {
                closeSinceNanos = 0L
                false
            }

            if (bumpPickup || settled) {
                collected++
                ballMemory.remove(target.id)
                committedId = null
                closeSinceNanos = 0L
                telemetry.addData(
                    "COLLECT pickup",
                    "${target.label} (%s)".format(if (bumpPickup) "current bump" else "settled"),
                )
                telemetry.update()
            }
        }

        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        intake.stop()
        return collected
    }

    /**
     * PHASE 1b: optional flower use — a live sighting first, else a fresh
     * remembered spot from the shared BallMemory. Drives to
     * FLOWER_APPROACH_CM (turn-toward while approaching), stops, runs the
     * CUSTOMIZABLE flowerPullRoutine from globals.kt, then settles the intake.
     * Returns true when a pull was actually performed.
     */
    private fun useFlowerIfVisible(deadlineNanos: Long): Boolean {
        var positioned = false

        while (opModeIsActive() && System.nanoTime() < deadlineNanos) {
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)
            val now = System.nanoTime()

            ballMemory.absorb(ballLocator.visibleTargets(), drive.pose, now)
            report()

            val live = ballLocator.nearestFlower()
            val remembered = if (live == null) {
                ballMemory.bestFlower(drive.pose.position.x * 2.54, drive.pose.position.y * 2.54, now)
            } else {
                null
            }

            val distCm: Double
            val bearingDeg: Double
            val liveNow: Boolean
            if (live != null) {
                distCm = live.distanceCm
                bearingDeg = live.bearingDeg
                liveNow = true
            } else if (remembered != null) {
                val rel = ballMemory.relativeTo(remembered, drive.pose)
                distCm = rel.first
                bearingDeg = rel.second
                liveNow = false
            } else {
                break // nothing visible, nothing remembered: skip the phase
            }

            telemetry.addData(
                "FLOWER",
                "%.0fcm %+.1fdeg %s".format(distCm, bearingDeg, if (liveNow) "[live]" else "[memory]"),
            )
            telemetry.update()

            if (distCm <= FLOWER_APPROACH_CM + 5.0) {
                positioned = true
                break
            }

            val forward = (HiveScoringConfig.CAPTURE_RANGE_GAIN *
                (distCm - FLOWER_APPROACH_CM))
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val strafe = (-HiveScoringConfig.CAPTURE_BEARING_GAIN * bearingDeg)
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val turn = (-CAPTURE_TURN_GAIN * bearingDeg)
                .coerceIn(-CAPTURE_TURN_MAX_POWER, CAPTURE_TURN_MAX_POWER)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, strafe), turn))
        }

        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        if (!positioned) return false

        // ==== CUSTOMIZABLE PULL-OUT — edit flowerPullRoutine in globals.kt ====
        flowerPullRoutine(intake)

        // Give the released ball time to settle into the intake.
        intake.spinUp(1.0)
        sleep(FLOWER_SETTLE_AFTER_PULL_MS)
        intake.stop()
        return true
    }

    /**
     * PHASE 3 core: servo into the hive shooting window. Live tag first; when
     * the tag flickers out of view, dead-reckon toward the REMEMBERED hive
     * position (tag sightings continuously refresh it) instead of sweeping
     * blind. Returns true only when the live tag says ALIGNED — firing is
     * skipped otherwise.
     */
    private fun alignToHive(deadlineNanos: Long): Boolean {
        while (opModeIsActive() && System.nanoTime() < deadlineNanos) {
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)
            val now = System.nanoTime()

            val target = hive.update()
            tagOverlay.publish(target)
            report(target)

            if (target != null) {
                // Refresh the remembered hive position in field coordinates.
                val (hx, hy) = fieldPosOf(target.distanceCm, target.bearingDeg)
                hiveMemXcm = hx
                hiveMemYcm = hy
                hiveMemSeenNanos = now
            }

            if (target == null) {
                val hiveAgeSec = if (hiveMemSeenNanos == 0L) Double.MAX_VALUE else (now - hiveMemSeenNanos) / 1e9
                if (hiveAgeSec <= HIVE_MEMORY_MAX_AGE_SEC) {
                    // Dead-reckon toward the remembered hive: translate toward
                    // it while turning to sweep the tag back into view.
                    val dx = hiveMemXcm - drive.pose.position.x * 2.54
                    val dy = hiveMemYcm - drive.pose.position.y * 2.54
                    val distCm = hypot(dx, dy)
                    val bearingDeg = Math.toDegrees(drive.pose.heading.toDouble() - atan2(dy, dx))
                    val wrapped = ((bearingDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0

                    val forward = (HiveScoringConfig.RANGE_GAIN *
                        (distCm - HiveScoringConfig.RANGE_CENTER_CM))
                        .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
                    val turn = (-HiveScoringConfig.BEARING_GAIN * wrapped)
                        .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
                    telemetry.addData("ALIGN", "tag lost — hive memory %.0fcm %+.0fdeg".format(distCm, wrapped))
                    telemetry.update()
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, 0.0), turn))
                } else {
                    // Nothing remembered either: sweep in place for a tag.
                    drive.setDrivePowers(
                        PoseVelocity2d(Vector2d(0.0, 0.0), -HiveScoringConfig.SEARCH_TURN_POWER),
                    )
                }
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

    /**
     * PHASE 4 core: per-ball feeding. One spin-up, then ONE FEED PULSE PER
     * BALL (FLYWHEEL_FEED_TIME_MS) with the wheels re-verified at speed
     * between balls — trailing balls never launch into decelerating wheels.
     * Returns the number of balls actually fed (0 = fire safely skipped).
     */
    private fun fireBalls(ballCount: Int): Int {
        val target = hive.update()
        val rpm = hive.shootingRpmFor(target)
        flywheel.spinToRpm(rpm)

        val spinDeadline = System.nanoTime() +
            (clock.remainingSec().coerceAtMost(AUTONOMY_FIRE_RESERVE_SEC) * 1e9).toLong()
        while (opModeIsActive() && !flywheel.isAtSpeed(rpm) && System.nanoTime() < spinDeadline) {
            telemetry.addData("PHASE 4", "spinning up to %.0f RPM".format(rpm))
            telemetry.addData(
                "Measured L/R",
                "%.0f / %.0f RPM".format(flywheel.measuredLeftRpm(), flywheel.measuredRightRpm()),
            )
            telemetry.update()
        }
        if (!flywheel.isAtSpeed(rpm)) {
            // Never feed balls into slow wheels: that's a weak shot
            // (and a possible stuck ball). Safe-skip instead.
            telemetry.addData("PHASE 4", "SKIP FIRE — wheels never reached %.0f RPM".format(rpm))
            telemetry.update()
            return 0
        }

        var fired = 0
        for (i in 1..ballCount) {
            if (!opModeIsActive()) break
            // Re-verify at speed before EVERY ball (each ball slows the wheels).
            val recoverDeadline = System.nanoTime() +
                (clock.remainingSec().coerceAtMost(AUTONOMY_FIRE_RESERVE_SEC) * 1e9).toLong()
            while (opModeIsActive() && !flywheel.isAtSpeed(rpm) && System.nanoTime() < recoverDeadline) {
                telemetry.addData("PHASE 4", "recovering to %.0f RPM (ball %d/%d)".format(rpm, i, ballCount))
                telemetry.update()
            }
            if (!flywheel.isAtSpeed(rpm)) break

            telemetry.addData("PHASE 4", "FIRING ball %d/%d at %.0f RPM".format(i, ballCount, rpm))
            telemetry.update()
            intake.spinUp(1.0)
            sleep(FLYWHEEL_FEED_TIME_MS.toLong())
            intake.stop()
            fired++
        }
        return fired
    }

    /** Projects a robot-relative (distanceCm, bearingDeg) into field cm at the current pose. */
    private fun fieldPosOf(distCm: Double, bearingDeg: Double): Pair<Double, Double> {
        val px = drive.pose.position.x * 2.54
        val py = drive.pose.position.y * 2.54
        val h = drive.pose.heading.toDouble()
        val bRad = Math.toRadians(bearingDeg)
        val fwdCm = distCm * cos(bRad)
        val leftCm = -distCm * sin(bRad)
        return (px + fwdCm * cos(h) - leftCm * sin(h)) to (py + fwdCm * sin(h) + leftCm * cos(h))
    }

    /** Combined telemetry: hive range/angle, RR (fused) pose, vision health. */
    private fun report(target: HiveTarget? = null) {
        val t = target ?: hive.update()
        tagOverlay.publish(t)
        hive.addTelemetry(telemetry, t)

        val p = drive.pose
        telemetry.addData(
            "RR/FUSED pose",
            "(%.2f, %.2f) %.1f deg".format(p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble())),
        )
        vision.addTelemetry(telemetry)
        telemetry.update()
    }
}
