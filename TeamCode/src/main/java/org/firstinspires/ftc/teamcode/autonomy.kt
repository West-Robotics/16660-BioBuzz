package org.firstinspires.ftc.teamcode

/**
 * ============ AUTONOMY — the decision stack ============
 *
 * One merged home for everything that DECIDES (was shooter.kt,
 * WorldModel.kt, StrategicPlanner.kt, pathfinding.kt):
 *  * [ShooterModel]        — the calibrated RPM-vs-distance curve
 *  * [WorldModel]          — persistent field memory (tracks + hive)
 *  * [StrategicPlanner]     — lookahead + endgame value/cost brain
 *  * [StrategicController] — legacy reactive brain (Pathfinding Test only)
 */

import java.util.Locale
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.sin
// ============================================================================
// SECTION 1: ShooterModel (was shooter.kt)
// ============================================================================


/**
 * ============ SHOOTER MODEL — flywheel RPM vs. landing distance ============
 *
 * The calibration curve lives in globals.kt: [SHOOTER_CALIBRATION_RPM] paired
 * with [SHOOTER_CALIBRATION_DISTANCE_CM] (measured with the "Flywheel
 * Calibration" OpMode + tools/flywheel_analyzer.py). This object interpolates
 * that curve so the robot can ask "what flywheel RPM lands the ball at X cm?"
 * for ANY distance — the hive AprilTag estimator ([HiveScoring]) supplies X
 * live during matches.
 *
 * Until you fill in real measured distances, [isCalibrated] is false and every
 * query falls back to full speed (the old "spin at 100%" behavior), so the
 * robot is never left unable to shoot.
 */
object ShooterModel {

    /** Valid (rpm, distanceCm) samples, sorted by RPM ascending. */
    private fun samples(): List<Pair<Double, Double>> =
        SHOOTER_CALIBRATION_RPM.zip(SHOOTER_CALIBRATION_DISTANCE_CM)
            .filter { it.second > 0.0 }
            .sortedBy { it.first }

    /** True once at least two real calibration points exist. */
    fun isCalibrated(): Boolean = samples().size >= 2

    /** Shortest distance the curve covers (closest useful shooting distance). */
    fun minRangeCm(): Double = samples().minOfOrNull { it.second } ?: 0.0

    /** Longest distance the curve covers (the flywheel's physical reach). */
    fun maxRangeCm(): Double = samples().maxOfOrNull { it.second } ?: 0.0

    /**
     * Farthest distance we can shoot: the calibrated curve's reach, or the
     * HiveScoringConfig fallback window when no calibration data exists yet.
     */
    fun effectiveMaxRangeCm(): Double =
        if (isCalibrated()) maxRangeCm() else HiveScoringConfig.SHOOT_MAX_RANGE_CM

    /**
     * Flywheel RPM needed to land the ball [distanceCm] away. Linear
     * interpolation between the nearest calibrated samples; distances outside
     * the calibrated range clamp to the nearest end (too close still uses the
     * slowest calibrated RPM, too far uses the fastest). Uncalibrated -> 6000.
     */
    fun rpmForDistance(distanceCm: Double): Double {
        val s = samples()
        if (s.size < 2) return FLYWHEEL_MOTOR_MAX_RPM

        var lo = s.first()
        var hi = s.last()
        for (i in 0 until s.size - 1) {
            if (distanceCm in s[i].second..s[i + 1].second) {
                lo = s[i]
                hi = s[i + 1]
                break
            }
        }
        if (distanceCm <= lo.second) return lo.first
        if (distanceCm >= hi.second) return hi.first

        val t = (distanceCm - lo.second) / (hi.second - lo.second)
        return lo.first + t * (hi.first - lo.first)
    }

    /** Live summary for telemetry / the Driver Station. */
    fun summary(): String =
        if (isCalibrated()) {
            String.format(Locale.US, "calibrated %.0f-%.0fcm", minRangeCm(), maxRangeCm())
        } else {
            "UNCALIBRATED (fires full speed)"
        }
}

// ============================================================================
// SECTION 2: WorldModel (was WorldModel.kt)
// ============================================================================


/**
 * ============ WORLD MODEL — the robot's persistent field memory ============
 *
 * A human forgets a ball the moment it leaves view; the robot doesn't have to.
 * Every sighting (balls, flowers, opponent robots, the hive) is fused into
 * FIELD coordinates (when a pose is available) and kept as a track with:
 *   * position + velocity (for opponents),
 *   * a CONFIDENCE that decays smoothly while unseen and jumps back on
 *     re-sighting — objects are remembered for WM_*_MAX_AGE_SEC, never
 *     instantly forgotten.
 *
 * Coordinates: Road Runner field frame in CENTIMETERS (+x forward, +y left,
 * heading CCW, origin at the autonomous start pose). Without a pose (teleop
 * shadow mode) tracks live in a rolling robot-relative frame — association
 * still works, absolute memory doesn't.
 */
data class Sighting(
    val label: String,
    /** Straight-line distance from the robot, cm. */
    val distanceCm: Double,
    /** + = target is to the RIGHT of the robot centerline, degrees. */
    val bearingDeg: Double,
    val contourArea: Double,
    val isBall: Boolean,
)

data class TrackedObject(
    val id: Int,
    val label: String,
    var fieldXcm: Double,
    var fieldYcm: Double,
    var confidence: Double,
    var lastSeenNanos: Long,
    var velXcmPerSec: Double,
    var velYcmPerSec: Double,
    val isBall: Boolean,
    /** Latest sighting's contour area (px^2) — the chase-quality gate. */
    var lastAreaPx: Double = 0.0,
) {
    /** Game value of this object (POLLEN_VALUE / NECTAR_VALUE; 0 for others). */
    val value: Double
        get() = when {
            label.contains("Nectar") -> NECTAR_VALUE
            isBall -> POLLEN_VALUE
            else -> 0.0
        }

    fun distanceTo(xCm: Double, yCm: Double): Double = hypot(fieldXcm - xCm, fieldYcm - yCm)

    /** Seconds since the last visual sighting. */
    fun ageSec(nowNanos: Long): Double = (nowNanos - lastSeenNanos) / 1e9

    /**
     * Distance (cm) and bearing (deg, + = right of the robot centerline) of
     * this track from the robot — the same convention as live sightings, so
     * one set of pursuit gains serves visible and remembered targets alike.
     */
    fun relativeToRobot(robotXcm: Double, robotYcm: Double, headingRad: Double): Pair<Double, Double> {
        val dx = fieldXcm - robotXcm
        val dy = fieldYcm - robotYcm
        val bearing = wrapDeg(Math.toDegrees(headingRad) - Math.toDegrees(atan2(dy, dx)))
        return hypot(dx, dy) to bearing
    }
}

/** Wraps an angle in degrees into (-180, 180]. */
private fun wrapDeg(a: Double): Double = ((a + 180.0) % 360.0 + 360.0) % 360.0 - 180.0

class WorldModel {

    private val tracks = mutableListOf<TrackedObject>()
    private var nextId = 1

    // Hive memory (from the AprilTag estimator — the most trusted source).
    private var hiveXcm = 0.0
    private var hiveYcm = 0.0
    private var hiveConfidence = 0.0
    private var hiveSeenNanos = 0L
    private var lastUpdateNanos = 0L

    /**
     * Fuses the latest robot-relative [sightings] into the world model.
     * Pass the fused pose in cm ([poseXcm]/[poseYcm], [headingRad] CCW) when
     * available (autonomous / SelfPlay); pass nulls in pose-less teleop.
     */
    fun update(
        sightings: List<Sighting>,
        poseXcm: Double?,
        poseYcm: Double?,
        headingRad: Double,
        nowNanos: Long,
    ) {
        val dtSec = if (lastUpdateNanos == 0L) 0.0 else (nowNanos - lastUpdateNanos) / 1e9
        lastUpdateNanos = nowNanos

        if (dtSec > 0) {
            val decay = exp(-dtSec / WM_CONFIDENCE_TAU_SEC)
            for (t in tracks) t.confidence *= decay
        }

        val used = HashSet<Int>()
        for (s in sightings) {
            // robot-relative body frame: +forward, +left (bearing + = right)
            val bRad = Math.toRadians(s.bearingDeg)
            val fCm = s.distanceCm * cos(bRad)
            val lCm = -s.distanceCm * sin(bRad)

            // to field frame when we have a pose
            val xCm = if (poseXcm != null && poseYcm != null)
                poseXcm + fCm * cos(headingRad) - lCm * sin(headingRad) else fCm
            val yCm = if (poseXcm != null && poseYcm != null)
                poseYcm + fCm * sin(headingRad) + lCm * cos(headingRad) else lCm

            // nearest same-label track within the association radius
            var best: TrackedObject? = null
            var bestD = WM_TRACK_MATCH_CM
            for (t in tracks) {
                if (t.label != s.label || t.id in used) continue
                val d = hypot(t.fieldXcm - xCm, t.fieldYcm - yCm)
                if (d < bestD) {
                    best = t; bestD = d
                }
            }

            if (best != null) {
                val trackDt = (nowNanos - best.lastSeenNanos) / 1e9
                if (trackDt > 0.02 && poseXcm != null) {
                    best.velXcmPerSec = 0.7 * best.velXcmPerSec + 0.3 * (xCm - best.fieldXcm) / trackDt
                    best.velYcmPerSec = 0.7 * best.velYcmPerSec + 0.3 * (yCm - best.fieldYcm) / trackDt
                }
                // EMA-smooth the position so pinhole noise doesn't jerk the
                // track (the velocity estimate above still uses the raw delta).
                val a = WM_TRACK_EMA_ALPHA
                best.fieldXcm += a * (xCm - best.fieldXcm)
                best.fieldYcm += a * (yCm - best.fieldYcm)
                best.confidence = (best.confidence * 0.5 + 0.7).coerceAtMost(1.0)
                best.lastSeenNanos = nowNanos
                best.lastAreaPx = s.contourArea
                used.add(best.id)
            } else {
                tracks.add(
                    TrackedObject(
                        id = nextId++, label = s.label,
                        fieldXcm = xCm, fieldYcm = yCm,
                        confidence = 0.6, lastSeenNanos = nowNanos,
                        velXcmPerSec = 0.0, velYcmPerSec = 0.0,
                        isBall = s.isBall,
                        lastAreaPx = s.contourArea,
                    )
                )
            }
        }

        // forget stale / faded tracks
        tracks.removeAll { t ->
            val ageSec = (nowNanos - t.lastSeenNanos) / 1e9
            val maxAge = if (t.isBall) WM_BALL_MAX_AGE_SEC else WM_ROBOT_MAX_AGE_SEC
            ageSec > maxAge || t.confidence < 0.05
        }

        if (hiveSeenNanos != 0L && dtSec > 0) hiveConfidence *= exp(-dtSec / WM_CONFIDENCE_TAU_SEC)
    }

    /** Fuses the AprilTag hive measurement (already the best-range estimate). */
    fun updateHive(hive: HiveTarget?, poseXcm: Double?, poseYcm: Double?, headingRad: Double, nowNanos: Long) {
        val t = hive ?: return
        val bRad = Math.toRadians(t.bearingDeg)
        val fCm = t.distanceCm * cos(bRad)
        val lCm = -t.distanceCm * sin(bRad)
        if (poseXcm != null && poseYcm != null) {
            hiveXcm = poseXcm + fCm * cos(headingRad) - lCm * sin(headingRad)
            hiveYcm = poseYcm + fCm * sin(headingRad) + lCm * cos(headingRad)
        }
        hiveConfidence = if (hiveConfidence > 0) (hiveConfidence * 0.5 + 0.8).coerceAtMost(1.0) else 0.8
        hiveSeenNanos = nowNanos
    }

    /** Remembered balls, best-confidence first. */
    fun balls(): List<TrackedObject> = tracks.filter { it.isBall }.sortedByDescending { it.confidence }

    /** Lookup by track id (for target commitment in the COLLECT phase). */
    fun trackById(id: Int): TrackedObject? = tracks.firstOrNull { it.id == id }

    /** Forgets a ball (confirmed picked up) so it is never re-chased. */
    fun removeBall(id: Int) {
        tracks.removeAll { it.id == id && it.isBall }
    }

    /** Remembered flowers. */
    fun flowers(): List<TrackedObject> = tracks.filter { !it.isBall && it.label == "Flower" }

    /** Remembered opponent robots, freshest first. */
    fun opponents(): List<TrackedObject> =
        tracks.filter { !it.isBall && it.label != "Flower" }.sortedByDescending { it.confidence }

    /** Best remembered hive position (field cm), or null when never seen. */
    fun hive(): Pair<Double, Double>? =
        if (hiveConfidence > 0.1) hiveXcm to hiveYcm else null

    fun hiveConfidence(): Double = hiveConfidence

    fun clear() {
        tracks.clear()
        hiveConfidence = 0.0
        hiveSeenNanos = 0L
        lastUpdateNanos = 0L
    }

    /** Compact live view for the Driver Station. */
    fun summary(): String =
        "balls:%d flowers:%d opp:%d hive:%s".format(
            balls().size, flowers().size, opponents().size,
            if (hive() != null) "yes(%.0f%%)".format(hiveConfidence * 100) else "no",
        )

    /** Dumps the whole world model to the Black Box (called ~10 Hz). */
    fun logToBlackBox() {
        BlackBox.log(
            "record" to "wm",
            "balls" to balls().map {
                mapOf("id" to it.id, "x" to it.fieldXcm, "y" to it.fieldYcm,
                      "lbl" to it.label, "conf" to it.confidence)
            },
            "opp" to opponents().map {
                mapOf("id" to it.id, "x" to it.fieldXcm, "y" to it.fieldYcm,
                      "conf" to it.confidence, "vx" to it.velXcmPerSec, "vy" to it.velYcmPerSec)
            },
            "flowers" to flowers().map {
                mapOf("id" to it.id, "x" to it.fieldXcm, "y" to it.fieldYcm, "conf" to it.confidence)
            },
            "hiveX" to (if (hive() != null) hiveXcm else null),
            "hiveY" to (if (hive() != null) hiveYcm else null),
            "hiveConf" to hiveConfidence,
        )
    }
}
// ============================================================================
// SECTION 3: StrategicPlanner (was StrategicPlanner.kt)
// ============================================================================


/**
 * ============ STRATEGIC PLANNER — the lookahead brain ============
 *
 * Replaces the reactive "what do I see RIGHT NOW" strategy enum with a
 * value/cost search over the WorldModel's memory:
 *
 *  * every candidate action (collect ball i / go score / use flower / park)
 *    gets an expected value and an estimated time cost,
 *  * balls an opponent is closer to get discounted (they'll take them),
 *  * the ENDGAME is modeled: nothing starts that can't pay back before the
 *    clock runs out — full payloads force a score run, the last seconds
 *    force park.
 *
 * Deliberately small and deterministic (runs in microseconds) — the structure
 * is ready for a deeper search (MCTS) later, but this level already beats the
 * greedy controller because it remembers unseen balls and budgets time.
 *
 * It also powers SHADOW MODE: called during human teleop it logs what it
 * would do (without acting) — that log is how we'll prove it outplays the
 * human before ever letting it drive.
 */
enum class PlannerAction { COLLECT_BALL, GO_SCORE, USE_FLOWER, PARK, HOLD }

data class PlannerDecision(
    val action: PlannerAction,
    val targetId: Int,
    val targetLabel: String,
    val targetFieldXcm: Double,
    val targetFieldYcm: Double,
    val expectedValue: Double,
    val estCostSec: Double,
    val rationale: String,
)

class StrategicPlanner {

    /**
     * Picks the best action given the remembered world and our own state.
     *
     * @param world          persistent field memory (tracks + hive)
     * @param ballsHeld      balls currently in the robot
     * @param timeRemainingSec  seconds until the match ends — never start
     *                       anything that can't finish
     * @param robotXcm/Ycm   current robot position (field cm)
     */
    fun decide(
        world: WorldModel,
        ballsHeld: Int,
        timeRemainingSec: Double,
        robotXcm: Double,
        robotYcm: Double,
    ): PlannerDecision {
        val hive = world.hive()
        val distToHive =
            if (hive != null) hypot(hive.first - robotXcm, hive.second - robotYcm) else 150.0
        val scoreTime = distToHive / ROBOT_SPEED_CM_PER_SEC + PLANNER_SCORE_TOTAL_SEC

        // ---- 1. ENDGAME: the clock is the strongest constraint ----
        if (timeRemainingSec <= PLANNER_PARK_SEC + 2.0) {
            return PlannerDecision(
                PlannerAction.PARK, -1, "", 0.0, 0.0, 0.0, PLANNER_PARK_SEC,
                "PARK: %.0fs left".format(timeRemainingSec),
            )
        }
        if (ballsHeld > 0 && timeRemainingSec <= scoreTime + PLANNER_ENDGAME_MARGIN_SEC) {
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", 0.0, 0.0,
                ballsHeld * POLLEN_VALUE, scoreTime,
                "ENDGAME score run: %.0fs left, need %.1fs".format(timeRemainingSec, scoreTime),
            )
        }

        // ---- 2. FULL: no point holding more than we can carry ----
        if (ballsHeld >= MAX_BALLS_HELD && hive != null) {
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", hive.first, hive.second,
                ballsHeld * POLLEN_VALUE, scoreTime,
                "FULL (%d balls) -> score".format(ballsHeld),
            )
        }

        // ---- 3. Evaluate collect candidates (balls, then flowers) ----
        val opponents = world.opponents()
        var bestBall: TrackedObject? = null
        var bestDensity = 0.0
        var bestTime = 0.0
        var bestEffValue = 0.0
        for (b in world.balls()) {
            val dist = b.distanceTo(robotXcm, robotYcm)
            val t = dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_COLLECT_HANDLE_SEC
            var v = b.value * b.confidence
            // opponent closer to this ball than we are? they'll likely take it
            for (o in opponents) {
                if (o.distanceTo(b.fieldXcm, b.fieldYcm) < dist) {
                    v *= PLANNER_OPPONENT_DISCOUNT
                    break
                }
            }
            // can't collect AND still score it afterwards?
            if (timeRemainingSec < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
            val density = v / (1.0 + t)
            if (density > bestDensity) {
                bestDensity = density; bestBall = b; bestTime = t; bestEffValue = v
            }
        }

        var bestFlower: TrackedObject? = null
        var flowerDensity = 0.0
        var flowerTime = 0.0
        if (FLOWER_PULL_ENABLED && ballsHeld < MAX_BALLS_HELD) {
            for (fl in world.flowers()) {
                val dist = fl.distanceTo(robotXcm, robotYcm)
                val t = dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_FLOWER_TOTAL_SEC
                val v = POLLEN_VALUE * fl.confidence
                if (timeRemainingSec < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
                val density = v / (1.0 + t)
                if (density > flowerDensity) {
                    flowerDensity = density; bestFlower = fl; flowerTime = t
                }
            }
        }

        // Scoring what we hold is always an option; compare value densities.
        val scoreDensity =
            if (ballsHeld > 0) (ballsHeld * POLLEN_VALUE) / (1.0 + scoreTime) else 0.0

        // ---- 4. Choose ----
        val flowerWins =
            bestFlower != null && flowerDensity > bestDensity && flowerDensity > scoreDensity
        val ballWins = bestBall != null && bestDensity > scoreDensity
        if (ballsHeld > 0 && !ballWins && !flowerWins && hive != null) {
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", hive.first, hive.second,
                ballsHeld * POLLEN_VALUE, scoreTime,
                "score beats collecting (%.2f vs %.2f density)".format(scoreDensity, bestDensity),
            )
        }
        if (flowerWins) {
            return PlannerDecision(
                PlannerAction.USE_FLOWER, bestFlower!!.id, bestFlower.label,
                bestFlower.fieldXcm, bestFlower.fieldYcm, POLLEN_VALUE, flowerTime,
                "best density %.3f (flower)".format(flowerDensity),
            )
        }
        if (ballWins || (bestBall != null && ballsHeld == 0)) {
            return PlannerDecision(
                PlannerAction.COLLECT_BALL, bestBall!!.id, bestBall.label,
                bestBall.fieldXcm, bestBall.fieldYcm, bestEffValue, bestTime,
                "best density %.3f (val %.1f conf %.0f%%)".format(
                    bestDensity, bestBall.value, bestBall.confidence * 100,
                ),
            )
        }
        return PlannerDecision(
            PlannerAction.HOLD, -1, "", 0.0, 0.0, 0.0, 0.0,
            "nothing worth doing — hold & scan",
        )
    }

    /** Logs a decision to the Black Box with a source tag (real / shadow). */
    fun logDecision(d: PlannerDecision, source: String, ballsHeld: Int, timeRemainingSec: Double) {
        BlackBox.event(
            "planner_decision",
            "source" to source,
            "action" to d.action.name,
            "targetId" to d.targetId,
            "targetLabel" to d.targetLabel,
            "value" to d.expectedValue,
            "costSec" to d.estCostSec,
            "ballsHeld" to ballsHeld,
            "timeRemainingSec" to timeRemainingSec,
            "why" to d.rationale,
        )
    }
}
// ============================================================================
// SECTION 4: StrategicController (was pathfinding.kt — legacy, Pathfinding Test OpMode only)
// ============================================================================


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
     * hop. (The flywheel is injected by the constructing OpMode.)
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
