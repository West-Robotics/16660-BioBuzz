package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * ============ BALL MEMORY — persistent field-frame ball map ============
 *
 * A human forgets a ball the moment it leaves the camera; the robot doesn't
 * have to. Every sighting is projected into FIELD coordinates (Road Runner
 * pose, cm) and kept as a stable-ID entry with:
 *   * EMA-smoothed position/range (pinhole noise doesn't jerk the pursuit),
 *   * an age (unseen entries expire; only fresh ones are chased),
 *   * a "chaseable" contour-area gate (tiny/far blobs are logged but never
 *     worth leaving the current target for),
 *   * a `visibleNow` flag so the COLLECT loop can pick visual servo or
 *     dead-reckoned pursuit without caring which one applies.
 *
 * The COLLECT phase commits to one entry and only switches when another
 * beats it by TARGET_SWITCH_MARGIN — see BioBuzzAutonomous.collectBalls.
 *
 * Coordinates: Road Runner field frame in CENTIMETERS (+x forward, +y left,
 * heading CCW, origin at the autonomous start pose).
 */
class BallMemory {

    /** One remembered ball. */
    data class Entry(
        val id: Int,
        val label: String,
        var fieldXcm: Double,
        var fieldYcm: Double,
        var lastSeenNanos: Long,
        /** EMA of the live sighting range — used for pickup confirmation. */
        var smoothedDistCm: Double,
        var contourArea: Double,
        var visibleNow: Boolean,
    ) {
        /** Game value of this ball (globals.kt: POLLEN_VALUE / NECTAR_VALUE). */
        val value: Double get() = if (label.contains("Nectar")) NECTAR_VALUE else POLLEN_VALUE

        fun ageSec(nowNanos: Long): Double = (nowNanos - lastSeenNanos) / 1e9
    }

    private val entries = mutableListOf<Entry>()
    private var nextId = 1

    val size: Int get() = entries.size

    /**
     * Fuses this frame's visible [targets] (balls only; flowers ignored)
     * into the map. Re-sightings re-associate with the nearest same-label
     * entry within [BALL_MEMORY_MATCH_CM] and EMA-smooth it; new sightings
     * create entries. `visibleNow` is recomputed each call.
     */
    fun absorb(targets: List<BallTarget>, pose: Pose2d, nowNanos: Long) {
        for (e in entries) e.visibleNow = false

        // Road Runner pose is inches; the map is cm.
        val px = pose.position.x * 2.54
        val py = pose.position.y * 2.54
        val h = pose.heading.toDouble()

        for (t in targets) {
            if (t.label == "Flower") continue

            // robot-relative body frame (+fwd, +left; bearing + = right)
            val bRad = Math.toRadians(t.bearingDeg)
            val fwdCm = t.distanceCm * cos(bRad)
            val leftCm = -t.distanceCm * sin(bRad)
            val x = px + fwdCm * cos(h) - leftCm * sin(h)
            val y = py + fwdCm * sin(h) + leftCm * cos(h)

            val match = entries
                .filter { it.label == t.label }
                .minByOrNull { hypot(it.fieldXcm - x, it.fieldYcm - y) }
            if (match != null && hypot(match.fieldXcm - x, match.fieldYcm - y) <= BALL_MEMORY_MATCH_CM) {
                val a = BALL_MEMORY_EMA_ALPHA
                match.fieldXcm += a * (x - match.fieldXcm)
                match.fieldYcm += a * (y - match.fieldYcm)
                match.smoothedDistCm =
                    if (match.smoothedDistCm <= 0.0) t.distanceCm
                    else match.smoothedDistCm + a * (t.distanceCm - match.smoothedDistCm)
                match.contourArea = t.contourArea
                match.lastSeenNanos = nowNanos
                match.visibleNow = true
            } else {
                entries.add(
                    Entry(
                        id = nextId++, label = t.label,
                        fieldXcm = x, fieldYcm = y,
                        lastSeenNanos = nowNanos,
                        smoothedDistCm = t.distanceCm,
                        contourArea = t.contourArea,
                        visibleNow = true,
                    )
                )
            }
        }

        entries.removeAll { it.ageSec(nowNanos) > BALL_MEMORY_MAX_AGE_SEC }
    }

    /**
     * Distance (cm) and bearing (deg, + = right of the robot centerline) of
     * an entry from the robot's current pose — the same convention the live
     * BallTarget uses, so one set of pursuit gains drives both visual servo
     * and dead-reckoned pursuit.
     */
    fun relativeTo(entry: Entry, pose: Pose2d): Pair<Double, Double> {
        val px = pose.position.x * 2.54
        val py = pose.position.y * 2.54
        val dx = entry.fieldXcm - px
        val dy = entry.fieldYcm - py
        val dist = hypot(dx, dy)
        // World angle to the target minus our heading; +CCW error means the
        // target is LEFT, and bearing is +right, hence the negation.
        val bearing = wrapDeg(Math.toDegrees(pose.heading.toDouble() - atan2(dy, dx) * 180.0 / Math.PI))
        return dist to bearing
    }

    /** Chase utility of an entry (same formula as BallLocator.utilityOf). */
    fun utilityOf(entry: Entry, robotXcm: Double, robotYcm: Double): Double {
        val distCm = hypot(entry.fieldXcm - robotXcm, entry.fieldYcm - robotYcm)
        val visibility = if (entry.visibleNow) 1.0 else 0.75 // slightly prefer live targets
        return entry.value * PRIORITY_VALUE_WEIGHT *
            visibility / (1.0 + (distCm / 100.0) * PRIORITY_DISTANCE_WEIGHT)
    }

    /**
     * Best chase candidate (capacity-aware): fresh-enough, blob-solid
     * entries only; one slot left takes the highest-value ball outright
     * (LAST_SLOT_PREFER_HIGHEST_VALUE), otherwise highest utility.
     */
    fun best(robotXcm: Double, robotYcm: Double, remainingSlots: Int, nowNanos: Long): Entry? {
        if (remainingSlots <= 0) return null
        val candidates = entries.filter {
            it.ageSec(nowNanos) <= DEAD_RECKON_MAX_AGE_SEC && it.contourArea >= CHASE_MIN_CONTOUR_AREA_PX
        }
        if (candidates.isEmpty()) return null
        return if (remainingSlots == 1 && LAST_SLOT_PREFER_HIGHEST_VALUE) {
            candidates.maxWithOrNull(
                compareBy<Entry> { it.value }.thenBy { hypot(it.fieldXcm - robotXcm, it.fieldYcm - robotYcm) },
            )
        } else {
            candidates.maxByOrNull { utilityOf(it, robotXcm, robotYcm) }
        }
    }

    fun byId(id: Int): Entry? = entries.firstOrNull { it.id == id }

    /** Forgets a ball (called when it is confirmed picked up). */
    fun remove(id: Int) {
        entries.removeAll { it.id == id }
    }

    /** Compact live view for telemetry. */
    fun summary(nowNanos: Long): String =
        if (entries.isEmpty()) "empty"
        else entries.joinToString(" ") { e ->
            val age = if (e.visibleNow) "live" else "%.1fs".format(e.ageSec(nowNanos))
            "#%d:%s".format(e.id, age)
        }

    private fun wrapDeg(a: Double): Double = ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0
}

/**
 * The StrategicController is the "brain" of the robot during autonomous.
 * It maintains a map of the field, prioritizes balls, and decides when to switch
 * from collecting to scoring based on game logic and efficiency heuristics.
 */
class StrategicController(
    private val vision: VisionSystem,
    private val drivetrain: Drivetrain,
    private val flywheel: FlyWheel,
) {

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
    private val MAX_BALLS = MAX_BALLS_HELD
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
     * Preemptive flywheel spin-up. If we can see the hive, pre-spin to the RPM
     * the calibrated curve says our current distance needs; otherwise hold
     * the wheels at the PRIME speed so a later real spin-up is just a short
     * hop. (The flywheel is injected by the constructing OpMode — there are no
     * hardware singletons anymore; see the note in main.kt.)
     */
    private fun requestFlywheelSpinup() {
        val dist = hivePosition?.let { hypot(it.first, it.second) }
        flywheel.spinToRpm(dist?.let { ShooterModel.rpmForDistance(it) } ?: FLYWHEEL_PRIME_RPM)
    }

    /**
     * Calculates the relative position of the hive and converts it to a field coordinate.
     */
    private fun updateHivePosition(detections: List<VisionSystem.Detection>) {
        // "Hive Tag N" (single-tag library) or "Hive Cluster RED SCORING" (SDK 12.0 game library).
        val hiveTags = detections.filter { it.label.startsWith("Hive") }
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
        // Command the calibrated shooting speed for our current hive distance
        // (full speed if uncalibrated / hive lost). The match OpMode's fire
        // sequence only feeds balls once BOTH wheels are at speed
        // (FlyWheel.isAtSpeed) — this brain just picks the power level.
        val dist = hivePosition?.let { hypot(it.first, it.second) }
        flywheel.spinToRpm(dist?.let { ShooterModel.rpmForDistance(it) } ?: FLYWHEEL_MOTOR_MAX_RPM)

        // After the shot, reset counts.
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
