package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.blackbox.BlackBox
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.sin

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
) {
    /** Game value of this object (POLLEN_VALUE / NECTAR_VALUE; 0 for others). */
    val value: Double
        get() = when {
            label.contains("Nectar") -> NECTAR_VALUE
            isBall -> POLLEN_VALUE
            else -> 0.0
        }

    fun distanceTo(xCm: Double, yCm: Double): Double = hypot(fieldXcm - xCm, fieldYcm - yCm)
}

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
                best.fieldXcm = xCm
                best.fieldYcm = yCm
                best.confidence = (best.confidence * 0.5 + 0.7).coerceAtMost(1.0)
                best.lastSeenNanos = nowNanos
                used.add(best.id)
            } else {
                tracks.add(
                    TrackedObject(
                        id = nextId++, label = s.label,
                        fieldXcm = xCm, fieldYcm = yCm,
                        confidence = 0.6, lastSeenNanos = nowNanos,
                        velXcmPerSec = 0.0, velYcmPerSec = 0.0,
                        isBall = s.isBall,
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