package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.Gamepad

// NOTE: there are intentionally NO top-level hardware singletons anymore.
// NextMotor binds itself to the shared NextFTC event loop at construction,
// so eagerly constructing motors for devices that aren't configured would
// crash the app on a background thread (e.g. a one-wheel-flywheel test
// robot). Every OpMode now constructs exactly what its HardwareProbe says
// exists — see HardwareProbe.kt.


class Experiment : LinearOpMode() {
    override fun runOpMode() { // setup, code runs once
        // boot probe: no drive motors configured -> nothing to run
        val hw = HardwareProbe.probe(hardwareMap)
        if (!hw.driveMotors) {
            while (!isStopRequested) {
                telemetry.addLine("!! DRIVE MOTORS NOT DETECTED !!")
                telemetry.addLine("Check the Driver Station configuration.")
                telemetry.update()
            }
            return
        }

        // Local, probe-gated instances — see the note above the file.
        val drivetrain = Drivetrain()
        val teleOp = CustomTeleOp(drivetrain)
        val loader = Loader().also { it.home() }

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
            teleOp.refresh(controller)

            // Shooter/loader test bench: X (hold) indexes one ball per stroke
            // into the (possibly spinning) flywheel, Y (tap) homes the loader.
            if (controller.x) loader.stroke()
            if (controller.y) loader.home()
            telemetry.addData("LOADER", "phase %s, %d strokes".format(loader.phase.name, loader.strokeCount))
            telemetry.update()



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
