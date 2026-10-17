package org.firstinspires.ftc.teamcode
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import dev.nextftc.units.RotationsPerMinute
import dev.nextftc.units.radians
import dev.nextftc.units.rpm
import kotlin.math.roundToInt
import kotlin.math.sqrt
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Shared setup for one kind of goBILDA 5203-series Yellow Jacket (specs in
 * globals.kt). Sets the encoder counts->angle conversion (anglePerCount) so
 * closed-loop modes read TRUE wheel speed/position, plus sane starting gains:
 *
 *   * kV — throttle per (rad/s) of wheel speed; sized so kV * top speed ~= 1.0
 *   * kP — extra throttle per (rad/s) of velocity error (starts equal to kV)
 *   * kS — static-friction kick
 *
 * These are STARTING POINTS: tune them live (FTC Dashboard) or edit here if
 * the drivetrain lags or oscillates.
 */

/** Base position-PID kP used by NextFTC position moves; [Drivetrain.setSpeed] scales it. */
private const val POSITION_KP = 8.0

private fun nextYellowJacket(
    name: String,
    maxRpm: Double,
    encoderCpr: Double,
    configure: NextMotor.() -> Unit = {},
): NextMotor = NextMotor(
    name,
    anglePerCount = (2.0 * PI / encoderCpr).radians,
).apply {
    configure()
    val maxRadPerSec = maxRpm * 2.0 * PI / 60.0
    velocityConstants.apply {
        kV = 1.0 / maxRadPerSec   // full throttle at the motor's top speed
        kP = kV                    // start with P ~= V; raise if it lags
        kI = 0.0
        kS = 0.03                  // static-friction compensation
    }
    positionConstants.apply {
        kP = POSITION_KP           // throttle per radian of position error (TUNE)
        kD = 0.0
        kV = 1.0 / maxRadPerSec
    }
}

class Drivetrain : Mechanism {

    val FrontLeft = nextYellowJacket(DrivetrainFrontLeft, DRIVE_MOTOR_MAX_RPM, DRIVE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD // use FORWARD or REVERSE
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val FrontRight = nextYellowJacket(DrivetrainFrontRight, DRIVE_MOTOR_MAX_RPM, DRIVE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val BackLeft = nextYellowJacket(DrivetrainBackLeft, DRIVE_MOTOR_MAX_RPM, DRIVE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val BackRight = nextYellowJacket(DrivetrainBackRight, DRIVE_MOTOR_MAX_RPM, DRIVE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    fun cmToTicks(cm: Double): Int {
        return (cm / (Math.PI * WheelDiameterCm) * DRIVE_MOTOR_ENCODER_CPR * WheelGearRatio).roundToInt()
    }
    fun cmToRad(cm: Double): Double {
        return ((2.0 * cm / WheelDiameterCm) * WheelGearRatio)
    }

    /**
     * Commands each wheel to rotate by the given deltas (radians), measured
     * FROM ITS CURRENT POSITION. NextFTC position setpoints are ABSOLUTE in
     * the accumulated-encoder frame — issuing them without the current
     * position would target "N radians since hub boot", which is only a
     * relative move on a freshly zeroed encoder and never twice in a row.
     */
    private fun moveBy(dFl: Double, dFr: Double, dBl: Double, dBr: Double) {
        FrontLeft.setPositionSetpoint(FrontLeft.encoderPosition + dFl.radians)
        FrontRight.setPositionSetpoint(FrontRight.encoderPosition + dFr.radians)
        BackLeft.setPositionSetpoint(BackLeft.encoderPosition + dBl.radians)
        BackRight.setPositionSetpoint(BackRight.encoderPosition + dBr.radians)
    }

    /**
     * Moves the robot FORWARD [cm] centimeters relative to its current spot
     * (mecanum forward = all wheels forward). Re-issuing extends the move.
     * [speed] scales the position controller's kP (see [setSpeed]).
     */
    fun driveForward(cm: Double, speed: Double = 1.0) {
        val rad: Double = cmToRad(cm)
        setSpeed(speed)
        moveBy(rad, rad, rad, rad)
    }
    fun driveBackward(cm: Double, speed: Double = 1.0) {driveForward(-cm, speed)}

    fun driveRight(cm: Double, speed: Double = 1.0) {
        val wheelCm = cm * sqrt(2.0)
        val rad = cmToRad(wheelCm)

        setSpeed(speed)

        moveBy(rad, -rad, -rad, rad)
    }

    fun driveLeft(cm: Double, speed: Double = 1.0) {
        val wheelCm = cm * sqrt(2.0)
        val rad = cmToRad(wheelCm)
        setSpeed(speed)
        moveBy(-rad, rad, rad, -rad)
    }

    fun turn(degrees: Double, speed: Double = 1.0) {
        val radius = sqrt(
            WheelBaseCm * WheelBaseCm +
                    TrackWidthCm * TrackWidthCm
        ) / 2.0

        val angleRad = degrees * PI / 180.0
        val wheelCm = radius * angleRad
        val motorRad = cmToRad(wheelCm)

        setSpeed(speed)

        moveBy(motorRad, -motorRad, motorRad, -motorRad)
    }

    /**
     * Moves the robot at a specific angle relative to its current orientation for a given distance.
     * @param cm The distance to move in centimeters.
     * @param degrees The angle at which to move (0 is forward, 90 is right, 180 is backward, 270 is left).
     * @param speed The speed multiplier (0.0 to 1.0).
     */
    fun driveAtAngle(cm: Double, degrees: Double, speed: Double = 1.0) {
        val radAngle = degrees * PI / 180.0
        val forward = cm * cos(radAngle)
        val strafe = cm * sin(radAngle)

        val strafeMultiplier = sqrt(2.0)

        val flWheel = forward + strafe * strafeMultiplier
        val frWheel = forward - strafe * strafeMultiplier
        val blWheel = forward - strafe * strafeMultiplier
        val brWheel = forward + strafe * strafeMultiplier

        setSpeed(speed)

        moveBy(
            cmToRad(flWheel), cmToRad(frWheel),
            cmToRad(blWheel), cmToRad(brWheel),
        )
    }

    fun driveTeleOp(forward: Double, strafe: Double, turn: Double) {
        var frontLeft = forward + strafe + turn
        var frontRight = forward - strafe - turn
        var backLeft = forward - strafe + turn
        var backRight = forward + strafe - turn

        // Normalize so none exceeds 1.0
        val max = maxOf(
            1.0,
            kotlin.math.abs(frontLeft),
            kotlin.math.abs(frontRight),
            kotlin.math.abs(backLeft),
            kotlin.math.abs(backRight)
        )

        frontLeft /= max
        frontRight /= max
        backLeft /= max
        backRight /= max

        FrontLeft.setVelocitySetpoint((frontLeft * DRIVE_MOTOR_MAX_RPM).rpm)
        FrontRight.setVelocitySetpoint((frontRight * DRIVE_MOTOR_MAX_RPM).rpm)
        BackLeft.setVelocitySetpoint((backLeft * DRIVE_MOTOR_MAX_RPM).rpm)
        BackRight.setVelocitySetpoint((backRight * DRIVE_MOTOR_MAX_RPM).rpm)
    }


    /**
     * Speed scale for the NEXT position move (driveForward/turn/driveAtAngle).
     *
     * NextFTC position control has no separate target-velocity knob — a
     * position move's speed comes from the position PID's kP — so "speed"
     * here scales kP: 1.0 = full gain ([POSITION_KP], the tuned value),
     * 0.25 = a gentle quarter-gain crawl. Call it BEFORE the move (all move
     * helpers call it for you). Velocity control ([driveTeleOp]) is unaffected.
     *
     * (The old body was commented out, which silently ignored every `speed`
     * argument in this file — PathfindingTest's 0.5/0.8 approach speeds,
     * Experiment's scripted moves.)
     */
    fun setSpeed(speed: Double = 1.0) {
        val kp = POSITION_KP * speed.coerceIn(0.0, 1.0)
        FrontLeft.positionConstants.kP = kp
        FrontRight.positionConstants.kP = kp
        BackLeft.positionConstants.kP = kp
        BackRight.positionConstants.kP = kp
    }


    /**
     * Actually stops the drivetrain: switches every motor from position
     * control to a zero-velocity command (the setpoint persists otherwise —
     * the robot would keep driving to its last position target).
     */
    fun flushMotors(noCapSpeed: Boolean = true) {
        if (noCapSpeed) {
            FrontLeft.setVelocitySetpoint(0.0.rpm)
            FrontRight.setVelocitySetpoint(0.0.rpm)
            BackLeft.setVelocitySetpoint(0.0.rpm)
            BackRight.setVelocitySetpoint(0.0.rpm)
        }
    }
}
class Intake : Mechanism {
    val intakeMotor = nextYellowJacket(IntakeMotor, INTAKE_MOTOR_MAX_RPM, INTAKE_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD // use FORWARD or REVERSE
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT // use BRAKE or FLOAT
    }
    fun spinUp(speed: Double = 1.0) {
        // NOTE: full range -1..1 — reverse (gamepad B in teleop) is a real
        // reverse here; the old code clamped negatives to a stop.
        val velocity = speed.coerceIn(-1.0, 1.0) * INTAKE_MOTOR_MAX_RPM
        intakeMotor.setVelocitySetpoint(velocity.rpm)
    }
    fun stop(breakMotor: Boolean = false) {
        intakeMotor.zeroPowerBehavior =
                if (breakMotor)
                    NextMotor.ZeroPowerBehavior.BRAKE
                else
                    NextMotor.ZeroPowerBehavior.FLOAT

        intakeMotor.setVelocitySetpoint(0.0.rpm)

    }

    /** Present motor current draw in amps (0.0 when the read fails). */
    val currentAmps: Double
        get() = try {
            intakeMotor.current.magnitude
        } catch (_: Exception) {
            0.0
        }
}

/**
 * Detects a ball entering the intake by its CURRENT signature: a real load is
 * a brief current rise above the running baseline (and back down) — unlike a
 * jam, which stays high. Used by the COLLECT phase as physical pickup
 * confirmation, so ball counting no longer relies on settle/vanish inference.
 *
 * MEASURE PICKUP_CURRENT_BUMP_A on the real robot: with the intake spinning
 * empty, read telemetry baseline current; then feed a ball and read the peak.
 */
class PickupCurrentSensor(private val intake: Intake) {

    private var baselineAmps = 0.5
    private var highSinceNanos = 0L

    /**
     * Call every loop. [spinning] = whether the intake is being driven right
     * now (sensor idles otherwise). Returns true exactly once per completed
     * ball-load bump.
     */
    fun update(spinning: Boolean): Boolean {
        val now = System.nanoTime()
        if (!spinning) {
            highSinceNanos = 0L
            return false
        }

        val current = intake.currentAmps
        return if (current >= baselineAmps + PICKUP_CURRENT_BUMP_A) {
            if (highSinceNanos == 0L) highSinceNanos = now
            false
        } else {
            // Slowly track the empty-running baseline when current is low.
            baselineAmps += 0.05 * (current - baselineAmps)
            if (highSinceNanos == 0L) return false
            val bumpMs = (now - highSinceNanos) / 1e6
            highSinceNanos = 0L
            // A rise that lasted between min and max = one ball went in.
            bumpMs in PICKUP_BUMP_MIN_MS..PICKUP_BUMP_MAX_MS
        }
    }
}

/**
 * DUAL 6000 RPM flywheel: one 5203-series Yellow Jacket on each side of the
 * ball, spinning OPPOSITE physical directions (left FORWARD, right REVERSE)
 * so the ball is squeezed straight through. Both wheels run the same commanded
 * RPM, and encoder feedback ([isAtSpeed]) verifies they actually reached it
 * before anything fires.
 *
 * The 6000 RPM motors only make 1.47 kg.cm of torque, so spin-up from a dead
 * stop is slow — keep the wheels PRIMED ([prime]) whenever we hold balls, and
 * pre-spin to the speed the hive distance calls for (see [ShooterModel],
 * driven by [HiveScoring.shootingRpmFor]) while approaching.
 */
class FlyWheel : Mechanism {

    val flyWheelLeft = nextYellowJacket(FlywheelLeftMotor, FLYWHEEL_MOTOR_MAX_RPM, FLYWHEEL_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT
    }

    val flyWheelRight = nextYellowJacket(FlywheelRightMotor, FLYWHEEL_MOTOR_MAX_RPM, FLYWHEEL_MOTOR_ENCODER_CPR) {
        direction = NextMotor.Direction.REVERSE // opposite physical spin to the left wheel
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT
    }

    /** Holds both wheels at [rpm], clamped to the motor's real top speed. */
    fun spinToRpm(rpm: Double) {
        val target = rpm.coerceIn(0.0, FLYWHEEL_MOTOR_MAX_RPM)
        flyWheelLeft.setVelocitySetpoint(target.rpm)
        flyWheelRight.setVelocitySetpoint(target.rpm)
    }

    /** Legacy power-style control: 1.0 = full 6000 RPM, 0 = stopped. */
    fun spinUp(speed: Double = 1.0) {
        spinToRpm(speed.coerceIn(-1.0, 1.0) * FLYWHEEL_MOTOR_MAX_RPM)
    }

    /** Idles both wheels at [FLYWHEEL_PRIME_RPM] so a real shot is a short hop, not a long spin-up. */
    fun prime() {
        spinToRpm(FLYWHEEL_PRIME_RPM)
    }

    fun stop(breakMotor: Boolean = false) {
        flyWheelLeft.zeroPowerBehavior =
            if (breakMotor)
                NextMotor.ZeroPowerBehavior.BRAKE
            else
                NextMotor.ZeroPowerBehavior.FLOAT
        flyWheelRight.zeroPowerBehavior = flyWheelLeft.zeroPowerBehavior

        flyWheelLeft.setVelocitySetpoint(0.0.rpm)
        flyWheelRight.setVelocitySetpoint(0.0.rpm)
    }

    /** Measured LEFT wheel speed from the encoder, in RPM. */
    fun measuredLeftRpm(): Double = flyWheelLeft.encoderVelocity.into(RotationsPerMinute)

    /** Measured RIGHT wheel speed from the encoder, in RPM. */
    fun measuredRightRpm(): Double = flyWheelRight.encoderVelocity.into(RotationsPerMinute)

    /** Average measured speed of the wheels, RPM (used by [ShotDetector]). */
    fun measuredAverageRpm(): Double = (measuredLeftRpm() + measuredRightRpm()) / 2.0

    /**
     * True when BOTH wheels are within [toleranceRpm] of [targetRpm]. This is
     * the "safe to fire" check — launching on slow wheels is a short shot.
     */
    fun isAtSpeed(targetRpm: Double, toleranceRpm: Double = FLYWHEEL_SPEED_TOLERANCE_RPM): Boolean =
        kotlin.math.abs(measuredLeftRpm() - targetRpm) <= toleranceRpm &&
            kotlin.math.abs(measuredRightRpm() - targetRpm) <= toleranceRpm
}

/**
 * ============ SHOT DETECTOR — "did we actually shoot?" ============
 *
 * Confirms each fired ball from the FLYWHEEL ENCODERS: a ball hitting the
 * spinning wheels briefly loads them, so measured RPM dips and recovers.
 * Arm right after a feed pulse (intake) or loader stroke; a dip below
 * [SHOT_DIP_THRESHOLD_RPM] from the at-fire baseline within
 * [SHOT_DETECT_WINDOW_MS] = CONFIRMED, else MISSED.
 *
 * Core class (identical on both branches); the superhuman branch logs the
 * verdicts to the Black Box at the call site.
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

    /** Average measured speed of the wheels, RPM. */
    fun measuredRpm(): Double = flywheel.measuredAverageRpm()

    /** Arms dip detection. Call immediately after feeding a ball. */
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
        } else if (System.nanoTime() >= windowEndNanos) {
            state = State.MISSED
        }
        return state
    }

    fun totalConfirmed(): Int = confirmedCount

    fun idle() {
        state = State.IDLE
    }
}

