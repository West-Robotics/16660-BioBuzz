package org.firstinspires.ftc.teamcode
import dev.nextftc.hardware.actuators.NextMotor
import dev.nextftc.robot.Mechanism
import dev.nextftc.units.radians
import dev.nextftc.units.rpm
import kotlin.math.roundToInt
import kotlin.math.sqrt
import kotlin.math.PI

class drivetrain : Mechanism {

    val FrontLeft = NextMotor(DrivetrainFrontLeft).apply {
        direction = NextMotor.Direction.FORWARD // use FORWARD or REVERSE
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val FrontRight = NextMotor(DrivetrainFrontRight).apply {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val BackLeft = NextMotor(DrivetrainBackLeft).apply {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    val BackRight = NextMotor(DrivetrainBackRight).apply {
        direction = NextMotor.Direction.FORWARD
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE // use BRAKE or FLOAT
    }

    fun cmToTicks(cm: Double): Int {
        return (cm / (Math.PI * WheelDiameterCm) * ticksPerMotorRev * WheelGearRatio).roundToInt()
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


    fun setSpeed(speed: Double = 1.0) {
        val velocity = speed.coerceIn(0.0, 1.0) * maxRPM
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
class intake : Mechanism {
    val intakeMotor = NextMotor(IntakeMotor).apply {
        direction = NextMotor.Direction.FORWARD // use FORWARD or REVERSE
        zeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT // use BRAKE or FLOAT
    }
    fun runIntake(speed: Double = 1.0) {
        val velocity = speed.coerceIn(0.0, 1.0) * maxRPM
        intakeMotor.setVelocitySetpoint(velocity.rpm)
    }
    fun stopIntake(breakMotor: Boolean = false) {
        intakeMotor.zeroPowerBehavior =
                if (breakMotor)
                    NextMotor.ZeroPowerBehavior.BRAKE
                else
                    NextMotor.ZeroPowerBehavior.FLOAT

        intakeMotor.setVelocitySetpoint(0.0.rpm)

    }
}

