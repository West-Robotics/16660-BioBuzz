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
 *   3. ALIGN:   search for the hive tag, servo into the [HiveScoringConfig]
 *              shooting window; flywheel pre-spins while approaching.
 *   4. FIRE:    flywheel already spinning — hold FIRE_TIME_MS to launch.
 *   5. PARK:    back off from the hive and stop.
 *
 * Prereqs on the real robot: RR tuning (MecanumDrive.PARAMS), "imu" configured,
 * vision test procedure passed, [HiveScoringConfig] measured.
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

        telemetry.addLine("BioBuzz autonomous: RR + hybrid vision + AprilTag scoring")
        telemetry.addData(
            "Vision corrections",
            if (drive.visionCorrectionEnabled) "ENABLED" else "disabled (pass vision tests first)",
        )
        telemetry.addData(
            "Shoot window",
            "${HiveScoringConfig.SHOOT_MIN_RANGE_CM.toInt()}-" +
                "${HiveScoringConfig.SHOOT_MAX_RANGE_CM.toInt()} cm, " +
                "|angle| <= ${HiveScoringConfig.SHOOT_MAX_BEARING_DEG.toInt()} deg",
        )
        telemetry.update()

        waitForStart()

        try {
            // ---- PHASE 1: COLLECT (vision-directed, capacity-aware) ----
            telemetry.addLine("PHASE 1: vision-directed collection")
            val collected = collectBalls()
            telemetry.addData("PHASE 1 done", "$collected / $MAX_BALLS_HELD balls")

            // ---- PHASE 1b: FLOWER (optional, customizable pull-out) ----
            if (FLOWER_PULL_ENABLED && collected < MAX_BALLS_HELD) {
                telemetry.addLine("PHASE 1b: flower pull")
                useFlowerIfVisible()
            }

            // ---- PHASE 2: POLISH (vision-guided pose alignment) ----
            telemetry.addLine("PHASE 2: vision-guidance polish")
            runGuidance()

            // ---- PHASE 3: ALIGN (AprilTag window; flywheel pre-spin) ----
            telemetry.addLine("PHASE 3: hive alignment")
            flywheel.spinUp(1.0) // spin-up overlaps the approach
            val aligned = alignToHive()

            // ---- PHASE 4: FIRE ----
            telemetry.addData(
                "PHASE 4", if (aligned) "FIRING" else "ALIGN FAILED - skipping fire (safe)",
            )
            telemetry.update()
            if (aligned) {
                sleep(HiveScoringConfig.FIRE_TIME_MS.toLong())
            }
            flywheel.stop()

            // ---- PHASE 5: PARK ----
            telemetry.addLine("PHASE 5: park")
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
     * PHASE 1 core: capacity-aware, vision-directed collection. Picks the
     * PRIORITY ball (see globals.kt: PRIORITY_* / LAST_SLOT_*), drives onto it
     * with the intake running, confirms the pickup by proximity-settle or by
     * the target vanishing at the intake, then repeats until full or timeout.
     * Falls back to a slow forward creep when nothing is visible.
     * Bounded by AUTONOMOUS_COLLECT_TIMEOUT_MS.
     */
    private fun collectBalls(): Int {
        var collected = 0
        var lastTarget: BallTarget? = null
        var closeSinceNanos = 0L
        val deadline = System.nanoTime() + (AUTONOMOUS_COLLECT_TIMEOUT_MS * 1e6).toLong()

        intake.spinUp(1.0)
        while (opModeIsActive() && collected < MAX_BALLS_HELD && System.nanoTime() < deadline) {
            drive.updatePoseEstimate()
            vision.updateRoadRunnerPose(drive.pose)

            val best = ballLocator.bestBall(MAX_BALLS_HELD - collected)
            report()

            if (best == null) {
                // A target that vanished at close range was almost certainly
                // swallowed by the intake — count it.
                if (lastTarget != null && lastTarget.distanceCm <= PICKUP_CONFIRM_CM) {
                    collected++
                    telemetry.addData("COLLECT pickup", "${lastTarget.label} (vanished)")
                    closeSinceNanos = 0L
                } else {
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(0.15, 0.0), 0.0)) // creep
                }
                lastTarget = null
                continue
            }

            telemetry.addData(
                "COLLECT $collected/$MAX_BALLS_HELD",
                "%s %.0fcm %+.1fdeg".format(best.label, best.distanceCm, best.bearingDeg),
            )
            telemetry.update()

            // Steer toward the chosen ball (RR: +forward, +strafe left, +turn CCW).
            val forward = (HiveScoringConfig.CAPTURE_RANGE_GAIN *
                (best.distanceCm - HiveScoringConfig.CAPTURE_APPROACH_CM))
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            val strafe = (-HiveScoringConfig.CAPTURE_BEARING_GAIN * best.bearingDeg)
                .coerceIn(-HiveScoringConfig.ASSIST_MAX_POWER, HiveScoringConfig.ASSIST_MAX_POWER)
            drive.setDrivePowers(PoseVelocity2d(Vector2d(forward, strafe), 0.0))

            // Pickup confirmation 1: close for long enough.
            if (best.distanceCm <= PICKUP_CONFIRM_CM) {
                if (closeSinceNanos == 0L) {
                    closeSinceNanos = System.nanoTime()
                } else if ((System.nanoTime() - closeSinceNanos) / 1e6 >= PICKUP_SETTLE_MS) {
                    collected++
                    telemetry.addData("COLLECT pickup", "${best.label} (settled)")
                    closeSinceNanos = 0L
                    lastTarget = null
                    continue
                }
            } else {
                closeSinceNanos = 0L
            }
            lastTarget = best
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
        flowerPullRoutine()

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
        telemetry.addData(
            "RR/FUSED pose",
            "(%.2f, %.2f) %.1f deg".format(p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble())),
        )
        vision.addTelemetry(telemetry)
        telemetry.update()
    }
}
