package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

// NOTE: there are intentionally NO top-level hardware singletons anymore.
// NextMotor binds itself to the shared NextFTC event loop AT CONSTRUCTION, so
// eagerly constructing motors at class-load time (the old `val drivetrain =
// Drivetrain()` etc. here) constructs hardware for devices that may not be
// configured, crashing the app on a background thread the moment this file's
// globals are touched. Every OpMode constructs exactly what it needs, inside
// runOpMode(), where the hardware map is real.

/**
 * ============ EXPERIMENT — scripted drive test bench ============
 *
 * Registered under group "Test". Waits for START, then runs a short scripted
 * sequence of Drivetrain position moves (forward/back/left/right/turn/diagonal),
 * then hands control to the gamepad.
 */
@TeleOp(name = "Experiment (test bench)", group = "Test")
class Experiment : LinearOpMode() {
    override fun runOpMode() { // setup, code runs once
        // Local instances — see the note above this class.
        val drivetrain = Drivetrain()
        val teleOp = CustomTeleOp(drivetrain)

        telemetry.addLine("EXPERIMENT: waits for START, then runs the drive sequence.")
        telemetry.update()
        // Never move during INIT — the sequence only runs once the driver has
        // actually STARTed the OpMode. (The old code moved the robot the
        // moment it was selected on the Driver Station.)
        waitForStart()

        drivetrain.driveForward(10.0)
        sleep(5000)
        drivetrain.driveBackward(100.0)
        sleep(5000)
        drivetrain.driveRight(100.0)
        sleep(5000)
        drivetrain.driveLeft(100.0)
        sleep(5000)
        drivetrain.turn(90.0)
        sleep(5000)
        // Test new driveAtAngle function
        drivetrain.driveAtAngle(50.0, 45.0) // Diagonal 45 deg for 50 cm
        sleep(5000)
        drivetrain.driveAtAngle(60.0, 15.0) // 15 deg for 60 cm
        sleep(5000)
        drivetrain.flushMotors()

        while (opModeIsActive()) { // main loop
            // Use THIS OpMode's gamepad1: the old global `controller` captured
            // BlocksOpModeCompanion.gamepad1 at class-load time, which is only
            // ever populated by the Blocks runtime and never updates for a
            // Kotlin OpMode (null -> guaranteed NPE, or a dead stale object).
            teleOp.refresh(gamepad1)
        }
    }
}

/**
 * Minimal gamepad-to-drivetrain adapter used by the [Experiment] OpMode
 * (merged here from the old TeleOP.kt — it belongs with the robot singletons).
 */
class CustomTeleOp(
    private val drivetrain: Drivetrain
) {
    fun refresh(gamepad: Gamepad) {
        val forward = -gamepad.left_stick_y.toDouble()
        val strafe = gamepad.left_stick_x.toDouble()
        val turn = gamepad.right_stick_x.toDouble()

        drivetrain.driveTeleOp(forward, strafe, turn)
    }
}
