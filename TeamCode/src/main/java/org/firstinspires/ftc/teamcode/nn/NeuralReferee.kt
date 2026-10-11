package org.firstinspires.ftc.teamcode.nn

import org.firstinspires.ftc.teamcode.BallTarget
import org.firstinspires.ftc.teamcode.C270_FOCAL_LENGTH_PIXELS
import org.firstinspires.ftc.teamcode.FLOWER_PHYSICAL_WIDTH_MM
import org.firstinspires.ftc.teamcode.GROUND_CAM_WIDTH
import org.firstinspires.ftc.teamcode.NECTAR_PHYSICAL_WIDTH_MM
import org.firstinspires.ftc.teamcode.NN_DETECTOR_MODE
import org.firstinspires.ftc.teamcode.NN_LABELS
import org.firstinspires.ftc.teamcode.NN_REF_MARGIN
import org.firstinspires.ftc.teamcode.NN_REF_STALE_MS
import org.firstinspires.ftc.teamcode.POLLEN_PHYSICAL_WIDTH_MM
import org.firstinspires.ftc.teamcode.NnMode
import org.firstinspires.ftc.teamcode.blackbox.BlackBox
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.max

/**
 * ============ NEURAL REFEREE — the authority ladder's enforcement point ============
 *
 * Holds the detector's latest verdicts (published by [DetectorProcessor] at
 * NN_DETECT_HZ) and applies them to the classical HSV pipeline's targets
 * according to [NN_DETECTOR_MODE]:
 *
 *  * OFF / ADVISORY — HSV targets pass through untouched (advisory verdicts
 *    are visible on the DS preview + in the Black Box only).
 *  * REFEREE — while a FRESH verdict exists, any HSV blob the detector does
 *    NOT corroborate is removed (kills lighting hallucinations). If the
 *    verdict is stale, HSV is trusted as-is — the referee never acts blind.
 *  * PRIMARY — REFEREE behavior PLUS detector boxes no HSV blob matched are
 *    added as brand-new targets (the NN can truly drive the game).
 */
object NeuralReferee {

    data class Detection(
        val labelId: Int,
        val confidence: Double,
        /** Full-frame pixel box: left, top, width, height. */
        val x: Double,
        val y: Double,
        val w: Double,
        val h: Double,
    ) {
        val label: String get() = NN_LABELS.getOrElse(labelId) { "?" }
        val centerX: Double get() = x + w / 2
        val centerY: Double get() = y + h / 2
    }

    @Volatile
    private var latestList: List<Detection> = emptyList()

    @Volatile
    private var latestNanos = 0L

    /** Called by the DetectorProcessor after each inference (throttled inside). */
    fun publish(dets: List<Detection>, captureNanos: Long) {
        latestList = dets
        latestNanos = captureNanos
        BlackBox.log(
            "record" to "nn_det",
            "count" to dets.size,
            "det" to dets.map {
                mapOf("cls" to it.label, "conf" to it.confidence,
                      "x" to it.x, "y" to it.y, "w" to it.w, "h" to it.h)
            },
        )
    }

    fun latest(): List<Detection> = latestList

    /** True when the detector has spoken recently enough to referee. */
    fun fresh(): Boolean =
        System.nanoTime() - latestNanos < (NN_REF_STALE_MS * 1e6).toLong()

    /**
     * Applies the authority ladder to the HSV pipeline's targets. Called from
     * BallLocator.visibleTargets().
     */
    fun filter(targets: List<BallTarget>): List<BallTarget> {
        if (NN_DETECTOR_MODE == NnMode.OFF || NN_DETECTOR_MODE == NnMode.ADVISORY) {
            return targets
        }
        if (!fresh() || latestList.isEmpty()) {
            return targets // referee is asleep -> trust the classical pipeline
        }

        val kept = targets.filter { corroborated(it) }

        if (NN_DETECTOR_MODE == NnMode.REFEREE) {
            return kept
        }

        // PRIMARY: also add detector boxes the HSV pipeline never matched.
        val additions = latestList
            .filter { d -> d.labelId in NN_LABELS.indices && d.label != "?" }
            .filter { d -> targets.none { t -> sameLabel(d, t) && insideBox(t.screenX, t.screenY, d) } }
            .map { d -> d.toBallTarget() }
        return kept + additions
    }

    private fun corroborated(t: BallTarget): Boolean =
        latestList.any { d -> sameLabel(d, t) && insideBox(t.screenX, t.screenY, d) }

    private fun sameLabel(d: Detection, t: BallTarget): Boolean = d.label == t.label

    private fun insideBox(px: Double, py: Double, d: Detection): Boolean {
        val tx = d.w * (NN_REF_MARGIN - 1.0) / 2
        val ty = d.h * (NN_REF_MARGIN - 1.0) / 2
        return px > d.x - tx && px < d.x + d.w + tx &&
            py > d.y - ty && py < d.y + d.h + ty
    }

    /** Same pinhole math as BallLocator, for detector-added PRIMARY targets. */
    private fun Detection.toBallTarget(): BallTarget {
        val pixelWidth = max(w, h).coerceAtLeast(2.0)
        val widthMm = when (label) {
            "Pollen" -> POLLEN_PHYSICAL_WIDTH_MM
            "Flower" -> FLOWER_PHYSICAL_WIDTH_MM
            else -> NECTAR_PHYSICAL_WIDTH_MM
        }
        return BallTarget(
            label = label,
            distanceCm = C270_FOCAL_LENGTH_PIXELS * widthMm / pixelWidth / 10.0,
            bearingDeg = Math.toDegrees(atan2(centerX - GROUND_CAM_WIDTH / 2.0, C270_FOCAL_LENGTH_PIXELS)),
            screenX = centerX,
            screenY = centerY,
            contourArea = w * h,
        )
    }
}