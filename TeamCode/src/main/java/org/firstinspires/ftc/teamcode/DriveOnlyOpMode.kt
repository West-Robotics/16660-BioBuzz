package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.util.Locale

/**
 * ============ DRIVE ONLY — the minimal bring-up OpMode ============
 *
 * Runs on NOTHING but the Robot Controller (hub) + the 4 drive motors:
 * no cameras, no IMU, no servos, no Road Runner, no vision — nothing that
 * needs extra configured hardware. This is the first thing to run when the
 * robot is freshly wired, and the fastest loop to verify motor wiring,
 * direction and encoder sign.
 *
 * Controls:
 *   LEFT STICK drive / RIGHT STICK turn   (mecanum, same math as teleop)
 *   LEFT BUMPER (hold)   slow mode (30% power)
 *   dpad UP/DOWN         nudge forward/back a few cm (encoder position moves)
 *
 * Telemetry shows live wheel positions + velocities so you can confirm each
 * motor spins the right way and its encoder counts up.
 */
@TeleOp(name = "Drive Only (hub + motors)", group = "Test")
class DriveOnlyOpMode : LinearOpMode() {

    override fun runOpMode() {
        // Touches ONLY frontleft/frontright/backleft/backright.
        val drivetrain = Drivetrain()

        telemetry.addLine("DRIVE ONLY: hub + 4 drive motors, nothing else.")
        telemetry.addLine("LEFT STICK drive / RIGHT STICK turn")
        telemetry.addLine("LB slow mode | dpad UP/DOWN encoder nudge")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            val forward = -gamepad1.left_stick_y.toDouble()
            val strafe = gamepad1.left_stick_x.toDouble()
            val turn = gamepad1.right_stick_x.toDouble()
            val scale = if (gamepad1.left_bumper) 0.3 else 1.0

            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                // small relative encoder move — also proves position control
                val cm = if (gamepad1.dpad_up) 5.0 else -5.0
                drivetrain.driveForward(cm, speed = 0.25)
                sleep(400)
            } else {
                drivetrain.driveTeleOp(forward * scale, strafe * scale, turn * scale)
            }

            val motors = listOf(
                "FL" to drivetrain.FrontLeft, "FR" to drivetrain.FrontRight,
                "BL" to drivetrain.BackLeft, "BR" to drivetrain.BackRight,
            )
            for (name in motors) {
                val (label, m) = name
                val vel = try {
                    m.encoderVelocity.into(dev.nextftc.units.RotationsPerMinute)
                } catch (_: Exception) {
                    0.0
                }
                val pos = try {
                    m.encoderPosition.into(dev.nextftc.units.Radians)
                } catch (_: Exception) {
                    0.0
                }
                telemetry.addData(
                    label,
                    String.format(Locale.US, "%.0f RPM, %.1f rad", vel, pos),
                )
            }
            telemetry.addData("slow", if (gamepad1.left_bumper) "ON" else "off")
            telemetry.update()
        }

        drivetrain.driveTeleOp(0.0, 0.0, 0.0) // stop
    }
}