package org.firstinspires.ftc.teamcode

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

/**
 * The StrategicController is the "brain" of the robot during autonomous.
 * It maintains a map of the field, prioritizes balls, and decides when to switch
 * from collecting to scoring based on game logic and efficiency heuristics.
 */
class StrategicController(private val vision: VisionSystem, private val drivetrain: Drivetrain) {

    // 1. STATE
    private var pollenCount = 0
    private var nectarCount = 0
    private var unknownBallCount = 0
    private var hivePosition: Pair<Double, Double>? = null // (X, Y) in cm relative to last known origin

    enum class Strategy { COLLECT_BALLS, GO_TO_HIVE, SHOOT, AVOID_ROBOT }
    private var currentStrategy = Strategy.COLLECT_BALLS

    // This is our "plan" for the next action
    data class Target(
        val label: String,
        val angleDeg: Double,
        val distanceCm: Double
    )
    private var currentTarget: Target? = null

    // 2. CONSTANTS (from globals.kt)
    private val MAX_BALLS = 3
    private val HIVE_RANGE_CM = 60.0 // If closer than this, just score

    // ===== Ball Counting API =====
    // Anything that detects a ball entering the robot (vision intake zone,
    // beam-break sensor, touch sensor, etc.) calls one of these.
    // UNKNOWN is used when the source cannot identify the type (e.g. beam-break).
    fun onBallCollected(type: BallType) {
        when (type) {
            BallType.POLLEN -> pollenCount++
            BallType.NECTAR -> nectarCount++
            BallType.UNKNOWN -> unknownBallCount++
        }
    }

    // Convenience overload for vision-based sources that know the label.
    fun onBallCollected(label: String) {
        when {
            label.contains("Pollen") -> pollenCount++
            label.contains("Nectar") -> nectarCount++
            else -> unknownBallCount++
        }
    }

    /** Total balls currently held, regardless of type. */
    fun ballCount(): Int = pollenCount + nectarCount + unknownBallCount

    /** How many Pollen (yellow) balls are held. */
    fun pollenCount(): Int = pollenCount

    /** How many Nectar (red/blue) balls are held. */
    fun nectarCount(): Int = nectarCount

    /** How many balls of unidentified type are held. */
    fun unknownBallCount(): Int = unknownBallCount

    /** Clears all ball counts (e.g. after firing a full payload). */
    fun resetBallCount() {
        pollenCount = 0
        nectarCount = 0
        unknownBallCount = 0
    }

    /**
     * Main update loop. Called every cycle by the OpMode.
     * 1. Update field position from AprilTags.
     * 2. Check for enemy robots.
     * 3. Re-evaluate strategy.
     * 4. Issue drive commands.
     */
    fun update() {
        val detections = vision.getAllDetections()
        updateHivePosition(detections)

        // Preemptive flywheel: if shooting is *plausible*, keep the flywheel hot.
        if (ballCount() >= FLYWHEEL_PRESPIN_BALL_COUNT || currentStrategy == Strategy.GO_TO_HIVE) {
            requestFlywheelSpinup()
        }

        when (currentStrategy) {
            Strategy.COLLECT_BALLS -> updateCollection(detections)
            Strategy.GO_TO_HIVE -> updateTravelToHive()
            Strategy.SHOOT -> performShot()
            Strategy.AVOID_ROBOT -> handleAvoidance()
        }
    }

    /**
     * Placeholder hook for the flywheel mechanism.
     * When the real flywheel mechanism exists, it should expose a `warmUp()` function.
     */
    private fun requestFlywheelSpinup() {
        // TODO: flywheel.warmUp() once the mechanism is built.
        // For now, this is a no-op that reserves the integration point.
    }

    /**
     * Calculates the relative position of the hive and converts it to a field coordinate.
     */
    private fun updateHivePosition(detections: List<VisionSystem.Detection>) {
        val hiveTags = detections.filter { it.label.startsWith("Hive Tag") }
        if (hiveTags.isNotEmpty()) {
            val tag = hiveTags.first()
            // Convert the camera-relative pose to a global target.
            // For simplicity, we treat the camera pose as the field pose.
            // A real implementation would use full odometry.
            hivePosition = Pair(tag.xMm / 10.0, tag.yMm / 10.0)
        }
    }

    /**
     * Heuristic: Finds the best ball to chase.
     * Score = (Alliance Bonus) / (Distance + 1).
     * This is a greedy algorithm: it picks the single best target, not brute-forcing all paths.
     */
    private fun findBestBall(detections: List<VisionSystem.Detection>): VisionSystem.Detection? {
        val balls = detections.filter { it.label.contains("Pollen") || it.label.contains("Nectar") }
        if (balls.isEmpty()) return null

        return balls.maxByOrNull { ball ->
            // Closer balls are better (distance in cm)
            val distanceScore = 1.0 / (ball.distanceMm / 10.0 + 1.0)
            // Nectar is worth more points
            val allianceBonus = when {
                ball.label.contains("Nectar") -> NECTAR_SCORE_MULTIPLIER
                else -> 1.0
            }
            distanceScore * allianceBonus
        }
    }

    /**
     * LOGIC: Decide if we should keep collecting or switch to scoring.
     */
    private fun updateCollection(detections: List<VisionSystem.Detection>) {
        val ballCount = ballCount()

        // 1. Check Ball Count Trigger
        if (ballCount >= MAX_BALLS) {
            if (hivePosition != null) {
                currentStrategy = Strategy.GO_TO_HIVE
                return
            }
        }

        // 2. Check Proximity Trigger
        if (hivePosition != null) {
            val distToHive = hypot(hivePosition!!.first, hivePosition!!.second)
            if (distToHive < HIVE_RANGE_CM) {
                currentStrategy = Strategy.SHOOT
                return
            }
        }

        // 3. Check Efficiency Breakpoint
        // NOTE: The flywheel spins up *in parallel* with travel (preemptive spin-up),
        // so spin-up time is NOT added as a serial cost here.
        // If we have balls and the next ball takes longer to reach than the hive does, go score.
        val bestBall = findBestBall(detections)
        if (hivePosition != null) {
            val timeToHive = hypot(hivePosition!!.first, hivePosition!!.second) / ROBOT_SPEED_CM_PER_SEC

            if (bestBall != null) {
                val timeToBall = (bestBall.distanceMm / 10.0) / ROBOT_SPEED_CM_PER_SEC
                if (ballCount > 0 && timeToBall > timeToHive) {
                    currentStrategy = Strategy.GO_TO_HIVE
                    return
                }
            }
        }

        // 4. Check for Enemy Robots
        val enemyRobots = detections.filter { it.label.contains("Robot") }
        if (enemyRobots.isNotEmpty()) {
            if (bestBall != null && enemyRobots.any { isRobotInPath(it, bestBall) }) {
                currentTarget = null // Invalidate target to force a re-route next cycle
            }
        }

        // 5. Continue Collecting
        val target = currentTarget ?: bestBall?.let {
            Target(it.label, it.angleDegrees, it.distanceMm / 10.0)
        }

        if (target != null) {
            currentTarget = target
            // Use driveAtAngle to move towards the target
            // Note: driveAtAngle moves relative to robot; we assume robot faces forward for now.
            drivetrain.driveAtAngle(target.distanceCm, target.angleDeg, 0.5)
        } else if (hivePosition != null) {
            // No balls visible, might as well head to the hive if we have some
            if (ballCount > 0) currentStrategy = Strategy.GO_TO_HIVE
        }
    }

    private fun updateTravelToHive() {
        val hive = hivePosition
        if (hive != null) {
            val dist = hypot(hive.first, hive.second)
            val angle = atan2(hive.first, hive.second) * 180.0 / Math.PI

            if (dist < HIVE_RANGE_CM) {
                currentStrategy = Strategy.SHOOT
            } else {
                // Drive to the hive
                drivetrain.driveAtAngle(dist, angle, 0.8)
            }
        } else {
            // Lost sight of hive, fall back to collecting
            currentStrategy = Strategy.COLLECT_BALLS
        }
    }

    private fun performShot() {
        // Flywheel firing control would go here.
        // TODO: flywheel.fireBalls(ballCount()) once the mechanism is built.
        // After shooting, reset counts.
        resetBallCount()
        currentStrategy = Strategy.COLLECT_BALLS
    }

    private fun handleAvoidance() {
        // Simple avoidance: re-activate collection logic which will re-evaluate targets.
        currentStrategy = Strategy.COLLECT_BALLS
    }

    private fun isRobotInPath(robot: VisionSystem.Detection, ball: VisionSystem.Detection): Boolean {
        // Simple check: if the robot is close to the line between us and the ball.
        val angleDiff = abs(robot.angleDegrees - ball.angleDegrees)
        val robotDist = robot.distanceMm / 10.0
        val ballDist = ball.distanceMm / 10.0
        return angleDiff < 15.0 && robotDist < ballDist
    }

    fun getTelemetry(): String {
        return "Strategy: $currentStrategy | Balls: ${ballCount()} (P:$pollenCount N:$nectarCount U:$unknownBallCount) | Target: ${currentTarget?.label ?: "None"}"
    }
}
