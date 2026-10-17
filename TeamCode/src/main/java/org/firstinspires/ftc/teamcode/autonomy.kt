package org.firstinspires.ftc.teamcode

/**
 * ============ AUTONOMY CORE — shared by master and the superhuman branch ============
 *
 * The shared decision substrate (kept identical in both branches; the
 * superhuman branch adds BlackBox logging, the StrategicPlanner, neural
 * nets, reflexes and SelfPlay ON TOP of this file):
 *  * [PhaseClock]   — the 30s autonomous time budget
 *  * [ShooterModel] — the calibrated RPM-vs-distance curve
 *  * [WorldModel]   — persistent field memory (tracks + hive)
 *
 * Pure Kotlin — no Android imports — so the JVM unit tests
 * (src/test/java) can exercise all of it directly.
 */

import java.util.Locale
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.sin

// ============================================================================
// SECTION 1: PhaseClock — the 30s autonomous budget manager
// ============================================================================


/**
 * The phase machine's own timeouts can sum to more than the autonomous
 * period (9s collect + 6s flower + 4s polish + 8s align + fire > 30s), which
 * historically meant the FMS could kill the OpMode mid-fire and un-parked.
 * Every phase now asks the clock for a deadline instead of trusting its own
 * timeout: budgets are clamped to what remains after the reserved work
 * (park, fire), and the score loop asks [fits] before starting a cycle that
 * cannot finish.
 */
class PhaseClock(
    private val periodSec: Double = AUTONOMY_PERIOD_SEC,
    private val nowNanos: () -> Long = System::nanoTime,
) {
    private val startNanos = nowNanos()

    fun elapsedSec(): Double = (nowNanos() - startNanos) / 1e9

    /** Seconds left in the autonomous period. */
    fun remainingSec(): Double = (periodSec - elapsedSec()).coerceAtLeast(0.0)

    /** True when at least [sec] seconds remain in the period. */
    fun fits(sec: Double): Boolean = remainingSec() >= sec

    /**
     * Deadline (nanos) for a phase with its own [phaseTimeoutMs] cap: never
     * more than the phase wants, never more than the period has left after
     * [reserveSec] is set aside for later work. Zero-length means "start
     * nothing" (the deadline is now).
     */
    fun deadlineNanos(phaseTimeoutMs: Double, reserveSec: Double = 0.0): Long {
        val availableMs = (remainingSec() - reserveSec) * 1e3
        if (availableMs <= 0.0) return nowNanos()
        return nowNanos() + (minOf(phaseTimeoutMs, availableMs) * 1e6).toLong()
    }
}

// ============================================================================
// SECTION 2: ShooterModel (was shooter.kt)
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
// SECTION 3: WorldModel (was WorldModel.kt)
// ============================================================================

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

    /**
     * Serializable snapshot of the whole model — the Black-Box recorder (the
     * superhuman branch) logs this ~10 Hz; core master never calls it.
     */
    fun logPayload(): Map<String, Any?> = mapOf(
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

    /**
     * Chase utility of a ball track (same shape as the old BallLocator
     * utility, confidence-weighted; live sightings beat stale ones).
     */
    fun chaseUtilityOf(track: TrackedObject, robotXcm: Double, robotYcm: Double, nowNanos: Long): Double {
        val distCm = track.distanceTo(robotXcm, robotYcm)
        val visibility = if (track.ageSec(nowNanos) <= WM_VISIBLE_FRESH_SEC) 1.0 else 0.75
        return track.value * track.confidence * PRIORITY_VALUE_WEIGHT *
            visibility / (1.0 + (distCm / 100.0) * PRIORITY_DISTANCE_WEIGHT)
    }

    /**
     * Capacity-aware chase pick for the COLLECT phase (the core target
     * selector; the superhuman branch's StrategicPlanner overrides this
     * with lookahead + opponent discounting):
     *  - fresh-enough sighting of a solid-enough blob only,
     *  - exactly one slot left -> highest-value ball outright when
     *    LAST_SLOT_PREFER_HIGHEST_VALUE is set, else highest utility.
     */
    fun bestBallToChase(robotXcm: Double, robotYcm: Double, remainingSlots: Int, nowNanos: Long): TrackedObject? {
        if (remainingSlots <= 0) return null
        val candidates = balls().filter {
            it.ageSec(nowNanos) <= DEAD_RECKON_MAX_AGE_SEC && it.lastAreaPx >= CHASE_MIN_CONTOUR_AREA_PX
        }
        if (candidates.isEmpty()) return null
        return if (remainingSlots == 1 && LAST_SLOT_PREFER_HIGHEST_VALUE) {
            candidates.maxWithOrNull(
                compareBy<TrackedObject> { it.value }.thenBy { it.distanceTo(robotXcm, robotYcm) },
            )
        } else {
            candidates.maxByOrNull { chaseUtilityOf(it, robotXcm, robotYcm, nowNanos) }
        }
    }
}
