package org.firstinspires.ftc.teamcode

import dev.nextftc.hardware.actuators.NextServo
import dev.nextftc.robot.Mechanism
import org.firstinspires.ftc.teamcode.blackbox.BlackBox

/**
 * ============ BALL LOADER — the indexer between hopper and flywheel ============
 *
 * A goBILDA dual-mode servo in POSITION mode (config name [LoaderServoName])
 * meters exactly ONE ball per stroke. Firing is now deterministic loader
 * strokes — the intake does NOT run while shooting. All timings/positions live
 * in globals.kt ([LOADER_HOME_POSITION] … [LOADER_RETRACT_MS]); measure them
 * on the real robot.
 *
 * The stroke/retract cycle is time-based (standard PWM servos report no
 * position feedback to the hub), so [busy] is the source of truth for "is it
 * still moving" and [phase] settles automatically once the motion time ends.
 */
enum class LoaderPhase { HOME, STROKING, STROKED, RETRACTING }

class Loader : Mechanism {

    private val servo = NextServo(LoaderServoName)

    private var livePhase = LoaderPhase.HOME
    private var motionEndNanos = 0L

    /** Completed strokes since [resetCount] — the real "balls fed" counter. */
    var strokeCount = 0
        private set

    /** Current mechanical phase; settled lazily once motion time expires. */
    val phase: LoaderPhase
        get() {
            if (System.nanoTime() >= motionEndNanos) {
                livePhase = when (livePhase) {
                    LoaderPhase.STROKING -> LoaderPhase.STROKED
                    LoaderPhase.RETRACTING -> LoaderPhase.HOME
                    else -> livePhase
                }
            }
            return livePhase
        }

    /** True while the servo is still traveling. */
    val busy: Boolean
        get() = System.nanoTime() < motionEndNanos

    // NOTE: no auto-home in init — top-level singletons (main.kt) can be
    // constructed before the hardware map exists. OpModes call home() once
    // inside runOpMode(), where hardware is guaranteed ready.

    /** Commands the servo to the home (gate closed) position. */
    fun home() {
        servo.position = LOADER_HOME_POSITION
        livePhase = LoaderPhase.RETRACTING
        motionEndNanos = System.nanoTime() + (LOADER_RETRACT_MS * 1e6).toLong()
    }

    /**
     * Meters ONE ball into the flywheel (home -> stroke end).
     * Only works from HOME when not busy; returns false otherwise (safe to
     * call every loop tick — that's how "hold X to empty the hopper" works).
     */
    fun stroke(): Boolean {
        if (busy || phase != LoaderPhase.HOME) return false
        servo.position = LOADER_STROKE_END_POSITION
        livePhase = LoaderPhase.STROKING
        motionEndNanos = System.nanoTime() + (LOADER_STROKE_MS * 1e6).toLong()
        strokeCount++
        BlackBox.event("loader_stroke", "stroke" to strokeCount)
        return true
    }

    /**
     * Returns the servo home so the next stroke can index another ball.
     * (If your mechanism ratchets — indexing only on the forward stroke —
     * the retract is mechanically harmless by design; if not, set the stroke
     * positions so the cycle works for your hardware.)
     */
    fun retract(): Boolean {
        if (busy || phase != LoaderPhase.STROKED) return false
        servo.position = LOADER_HOME_POSITION
        livePhase = LoaderPhase.RETRACTING
        motionEndNanos = System.nanoTime() + (LOADER_RETRACT_MS * 1e6).toLong()
        return true
    }

    fun resetCount() {
        strokeCount = 0
    }
}

/**
 * ============ SHOT DETECTOR — "did we actually shoot?" ============
 *
 * Confirms each fired ball from the FLYWHEEL ENCODERS: a ball hitting the
 * spinning wheels briefly loads them, so measured RPM dips and recovers.
 * Arm right after a loader stroke completes; a dip below
 * [SHOT_DIP_THRESHOLD_RPM] from the at-fire baseline within
 * [SHOT_DETECT_WINDOW_MS] = CONFIRMED, else MISSED.
 *
 * Every verdict is written to the Black Box — those (distance, RPM, dip)
 * records are exactly what tools/shooter_refit.py learns the shooter curve
 * from after real matches.
 */
class ShotDetector(private val flywheel: FlyWheel) {

    enum class State { IDLE, ARMED, CONFIRMED, MISSED }

    var state = State.IDLE
        private set
    var lastDipRpm = 0.0
        private set

    private var baselineRpm = 0.0
    private var minRpm = Double.MAX_VALUE
    private var windowEndNanos = 0L
    private var confirmedCount = 0

    /** Average measured wheel speed (both wheels), RPM. */
    fun measuredRpm(): Double =
        (flywheel.measuredLeftRpm() + flywheel.measuredRightRpm()) / 2.0

    /** Arms dip detection. Call immediately after a loader stroke finishes. */
    fun arm() {
        baselineRpm = measuredRpm()
        minRpm = Double.MAX_VALUE
        windowEndNanos = System.nanoTime() + (SHOT_DETECT_WINDOW_MS * 1e6).toLong()
        state = State.ARMED
    }

    /** Call every loop tick while ARMED. Transitions only via the dip/timeout. */
    fun update(): State {
        if (state != State.ARMED) return state
        val rpm = measuredRpm()
        if (rpm < minRpm) minRpm = rpm

        if (minRpm < baselineRpm - SHOT_DIP_THRESHOLD_RPM) {
            state = State.CONFIRMED
            lastDipRpm = baselineRpm - minRpm
            confirmedCount++
            BlackBox.event(
                "shot_confirmed",
                "dipRpm" to lastDipRpm,
                "baselineRpm" to baselineRpm,
                "minRpm" to minRpm,
            )
        } else if (System.nanoTime() >= windowEndNanos) {
            state = State.MISSED
            BlackBox.event(
                "shot_missed",
                "baselineRpm" to baselineRpm,
                "minRpm" to minRpm,
            )
        }
        return state
    }

    fun totalConfirmed(): Int = confirmedCount

    fun idle() {
        state = State.IDLE
    }
}
