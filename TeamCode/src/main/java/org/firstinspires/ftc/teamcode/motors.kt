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
        kP = 8.0                   // throttle per radian of position error (TUNE)
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

  fun driveForward(cm: Double, speed: Double = 1.0) { // speed in range of 0 to 1, 1 high 0 low
        val rad: Double = cmToRad(cm)
        setSpeed(speed)
        FrontLeft.setPositionSetpoint(rad.radians)
        FrontRight.setPositionSetpoint(rad.radians)
        BackLeft.setPositionSetpoint(rad.radians)
        BackRight.setPositionSetpoint(rad.radians)
    }
    fun driveBackward(cm: Double, speed: Double = 1.0) {driveForward(-cm, speed)}

    fun driveRight(cm: Double, speed: Double = 1.0) {
        val wheelCm = cm * sqrt(2.0)
        val rad = cmToRad(wheelCm)

        setSpeed(speed)

        FrontLeft.setPositionSetpoint(rad.radians)
        FrontRight.setPositionSetpoint((-rad).radians)
        BackLeft.setPositionSetpoint((-rad).radians)
        BackRight.setPositionSetpoint(rad.radians)
    }

    fun driveLeft(cm: Double, speed: Double = 1.0) {
        val wheelCm = cm * sqrt(2.0)
        val rad = cmToRad(wheelCm)
        setSpeed(speed)
        FrontLeft.setPositionSetpoint((-rad).radians)
        FrontRight.setPositionSetpoint(rad.radians)
        BackLeft.setPositionSetpoint(rad.radians)
        BackRight.setPositionSetpoint((-rad).radians)
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

        FrontLeft.setPositionSetpoint(motorRad.radians)
        FrontRight.setPositionSetpoint((-motorRad).radians)
        BackLeft.setPositionSetpoint(motorRad.radians)
        BackRight.setPositionSetpoint((-motorRad).radians)
    }

    /**
     * Drives the robot at a specific angle relative to its current orientation for a given distance.
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

        FrontLeft.setPositionSetpoint(cmToRad(flWheel).radians)
        FrontRight.setPositionSetpoint(cmToRad(frWheel).radians)
        BackLeft.setPositionSetpoint(cmToRad(blWheel).radians)
        BackRight.setPositionSetpoint(cmToRad(brWheel).radians)
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


    fun setSpeed(speed: Double = 1.0) {
        val velocity = speed.coerceIn(0.0, 1.0) * DRIVE_MOTOR_MAX_RPM
/**
        FrontLeft.setVelocitySetpoint(velocity.rpm)
        FrontRight.setVelocitySetpoint(velocity.rpm)
        BackLeft.setVelocitySetpoint(velocity.rpm)
        BackRight.setVelocitySetpoint(velocity.rpm)
        **/
    }


    fun flushMotors(noCapSpeed: Boolean = true) {
        if (noCapSpeed) setSpeed(0.0)


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

    /**
     * True when BOTH wheels are within [toleranceRpm] of [targetRpm]. This is
     * the "safe to fire" check — launching on slow wheels is a short shot.
     */
    fun isAtSpeed(targetRpm: Double, toleranceRpm: Double = FLYWHEEL_SPEED_TOLERANCE_RPM): Boolean =
        kotlin.math.abs(measuredLeftRpm() - targetRpm) <= toleranceRpm &&
            kotlin.math.abs(measuredRightRpm() - targetRpm) <= toleranceRpm
}

