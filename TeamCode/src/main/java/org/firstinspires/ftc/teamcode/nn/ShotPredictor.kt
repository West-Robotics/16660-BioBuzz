package org.firstinspires.ftc.teamcode.nn

import android.content.Context
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.SHOT_PREDICTOR_FILE
import org.firstinspires.ftc.teamcode.SHOT_PREDICTOR_MAX_ADJUST
import org.firstinspires.ftc.teamcode.SHOT_PREDICTOR_MIN_CONFIDENCE
import org.firstinspires.ftc.teamcode.SHOT_PREDICTOR_MODE
import org.firstinspires.ftc.teamcode.NnMode
import org.firstinspires.ftc.teamcode.blackbox.BlackBox
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import kotlin.math.exp
import kotlin.math.tanh

/**
 * ============ SHOT-OUTCOME PREDICTOR — a tiny MLP in pure Kotlin ============
 *
 * Neural net #1 of the staged plan (B): learns, from every logged shot, the
 * probability that a ball fired at (distance, RPM, battery voltage) actually
 * scores. Trained OFFLINE by tools/train_shot_predictor.py on your laptop
 * (numpy only), exported as a small JSON of weights, copied to the RC app's
 * storage at [SHOT_PREDICTOR_FILE], and run here with a few dozen multiply
 * adds — no TFLite, no latency, nothing to install on the robot.
 *
 * Authority ([SHOT_PREDICTOR_MODE], default ADVISORY):
 *  * ADVISORY — logs P(hit) to the Black Box + telemetry only. Run this for
 *    several matches to prove the model before flipping.
 *  * REFEREE  — when P(hit) is below SHOT_PREDICTOR_MIN_CONFIDENCE, nudges
 *    the commanded RPM in the direction the model's gradient says improves
 *    the odds, bounded to +/- SHOT_PREDICTOR_MAX_ADJUST (5%).
 */
object ShotPredictor {

    @Volatile
    var available = false
        private set

    private var loadedOnce = false
    private var w1: Array<DoubleArray> = emptyArray() // [hidden][3]
    private var b1 = DoubleArray(0)
    private var w2 = DoubleArray(0)                    // [hidden]
    private var b2 = 0.0
    private var mean = DoubleArray(3)
    private var std = DoubleArray(3)
    private var lastLogNanos = 0L

    /** Loads the model once per app run. Safe to call from every OpMode. */
    fun ensureLoaded(context: Context) {
        if (loadedOnce) return
        loadedOnce = true
        if (SHOT_PREDICTOR_MODE == NnMode.OFF) return
        try {
            val f = File(context.getExternalFilesDir(null), SHOT_PREDICTOR_FILE)
            if (!f.exists()) {
                BlackBox.event("nn_status", "component" to "shot_predictor", "status" to "no model file")
                return
            }
            val root = JSONObject(f.readText())
            mean = doubles(root.getJSONArray("mean"))
            std = doubles(root.getJSONArray("std"))
            val ja = root.getJSONArray("w1")
            w1 = Array(ja.length()) { doubles(ja.getJSONArray(it)) }
            b1 = doubles(root.getJSONArray("b1"))
            w2 = doubles(root.getJSONArray("w2"))
            b2 = root.getDouble("b2")
            available = true
            BlackBox.event(
                "nn_status", "component" to "shot_predictor",
                "status" to "loaded", "hidden" to w1.size,
            )
        } catch (e: Exception) {
            BlackBox.event(
                "nn_status", "component" to "shot_predictor",
                "status" to "error", "msg" to (e.message ?: "?"),
            )
        }
    }

    private fun doubles(ja: JSONArray): DoubleArray =
        DoubleArray(ja.length()) { ja.getDouble(it) }

    /**
     * P(this shot scores) for the given shot parameters, from the trained
     * model. Null when no model is loaded. Features are normalized with the
     * training set's mean/std before the forward pass.
     */
    fun pHit(distanceCm: Double, rpm: Double, batteryVolts: Double): Double? {
        if (!available) return null
        val x = doubleArrayOf(
            (distanceCm - mean[0]) / std[0],
            (rpm - mean[1]) / std[1],
            (batteryVolts - mean[2]) / std[2],
        )
        val h = DoubleArray(w1.size)
        for (i in w1.indices) {
            var z = b1[i]
            val row = w1[i]
            for (j in 0 until 3) z += row[j] * x[j]
            h[i] = tanh(z)
        }
        var z2 = b2
        for (i in h.indices) z2 += w2[i] * h[i]
        return 1.0 / (1.0 + exp(-z2))
    }

    /**
     * Latest battery voltage, in volts. The OpModes feed this from their
     * hardwareMap voltage sensor (they already read it for the Black Box);
     * defaults to a healthy 12V if nobody has reported yet.
     */
    @Volatile
    var lastKnownBatteryVolts = 12.0
        private set

    /** Reads + stores the hub battery voltage; returns it (0.0 if unreadable). */
    fun reportHardwareMapVolts(hardwareMap: HardwareMap): Double {
        val v = try {
            hardwareMap.voltageSensor.iterator().next().voltage
        } catch (_: Exception) {
            0.0
        }
        if (v > 0) lastKnownBatteryVolts = v
        return v
    }

    fun batteryVolts(): Double = lastKnownBatteryVolts

    /**
     * The opinionated API the fire paths call: returns the RPM to actually
     * command. ADVISORY logs P(hit) (throttled to ~1 Hz); REFEREE also nudges
     * the RPM when the model doesn't like the shot, bounded to
     * [SHOT_PREDICTOR_MAX_ADJUST]. Never changes anything with no model, no
     * distance, or a good-looking shot.
     */
    fun adjustRpm(rpm: Double, distanceCm: Double, source: String): Double {
        if (!available || distanceCm <= 0.0) return rpm
        val p = pHit(distanceCm, rpm, batteryVolts()) ?: return rpm

        // throttled advisory logging (this is called every loop tick)
        val now = System.nanoTime()
        if (now - lastLogNanos > 1_000_000_000L) {
            lastLogNanos = now
            BlackBox.log(
                "record" to "nn_shot",
                "p" to p, "rpm" to rpm, "distanceCm" to distanceCm,
                "batteryV" to batteryVolts(), "source" to source,
            )
        }

        if (SHOT_PREDICTOR_MODE != NnMode.REFEREE) return rpm
        if (p >= SHOT_PREDICTOR_MIN_CONFIDENCE) return rpm

        // direction the model says improves the odds (numeric gradient wrt rpm)
        val eps = rpm * 0.01
        val pUp = pHit(distanceCm, rpm + eps, batteryVolts()) ?: p
        val grad = (pUp - p) / eps
        if (grad == 0.0) return rpm
        val dir = if (grad > 0) 1.0 else -1.0
        val adjusted = rpm * (1.0 + dir * SHOT_PREDICTOR_MAX_ADJUST)
        BlackBox.event(
            "nn_shot_adjust",
            "p" to p, "rpmIn" to rpm, "rpmOut" to adjusted,
            "distanceCm" to distanceCm, "source" to source,
        )
        return adjusted
    }
}