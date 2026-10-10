package org.firstinspires.ftc.teamcode

import java.util.Locale

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
