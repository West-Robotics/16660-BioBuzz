package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import java.util.Locale

/**
 * ============ FLYWHEEL CALIBRATION — measure the shooter curve ============
 *
 * Walks the dual flywheel through 6000/5000/4000/3000/2000/1000 RPM while you
 * film each shot with a camera (a phone is fine). Then run
 * tools/flywheel_analyzer.py on the video — it tracks the ball automatically
 * and prints the measured landing distance for each RPM, formatted ready to
 * paste into globals.kt.
 *
 * HOW TO USE (full walkthrough in tools/README.md):
 *  1. Aim the robot straight down a clear runway. Put the camera SIDE-ON to
 *     the ball's flight path, and lay two tape marks on the floor a known
 *     distance apart (the analyzer uses them to convert pixels -> cm).
 *  2. Load a ball, run this OpMode, pick the RPM step (dpad UP/DOWN).
 *  3. Press A to spin the flywheel up; wait until telemetry says AT SPEED.
 *  4. Press X to feed ONE ball through the flywheel; film where it lands.
 *  5. Repeat for every RPM step (shoot in the SAME order as
 *     SHOOTER_CALIBRATION_RPM: 6000 down to 1000), then run the analyzer.
 */
@TeleOp(name = "Flywheel Calibration", group = "Test")
class FlywheelCalibration : LinearOpMode() {

    private val rpmSteps = SHOOTER_CALIBRATION_RPM.toList()
    private var stepIdx = 0
    private var fineRpm = 0.0
    private var spinning = false

    private val flywheel = FlyWheel()
    private val intake = Intake()

    // Edge-detection memory so one button press = exactly one action.
    private var lastDpadUp = false
    private var lastDpadDown = false
    private var lastDpadLeft = false
    private var lastDpadRight = false
    private var lastA = false
    private var lastB = false
    private var lastX = false

    private fun commandedRpm(): Double =
        (rpmSteps[stepIdx] + fineRpm).coerceIn(0.0, FLYWHEEL_MOTOR_MAX_RPM)

    override fun runOpMode() {
        while (!isStarted && !isStopRequested) {
            telemetry.addLine("FLYWHEEL CALIBRATION (${rpmSteps.size} RPM steps)")
            telemetry.addLine("DPAD UP/DOWN: pick RPM step   DPAD LEFT/RIGHT: fine +/-25")
            telemetry.addLine("A: toggle spin   B: stop   X: feed ONE ball")
            telemetry.addData("RPM now", String.format(Locale.US, "%.0f", rpmSteps[stepIdx]))
            telemetry.addData("Curve", ShooterModel.summary())
            telemetry.update()
            sleep(50) // don't busy-spin a core during INIT
        }

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !lastDpadUp && stepIdx < rpmSteps.size - 1) {
                stepIdx++; fineRpm = 0.0
            }
            if (gamepad1.dpad_down && !lastDpadDown && stepIdx > 0) {
                stepIdx--; fineRpm = 0.0
            }
            if (gamepad1.dpad_right && !lastDpadRight) fineRpm += 25.0
            if (gamepad1.dpad_left && !lastDpadLeft) fineRpm -= 25.0
            if (gamepad1.a && !lastA) spinning = !spinning
            if (gamepad1.b && !lastB) spinning = false

            val rpm = commandedRpm()
            if (spinning) flywheel.spinToRpm(rpm) else flywheel.stop()

            // Feed exactly one ball: run the intake briefly while filming.
            if (gamepad1.x && !lastX) {
                intake.spinUp(1.0)
                sleep(FLYWHEEL_FEED_TIME_MS.toLong())
                intake.stop()
            }

            telemetry.addData("Commanded", String.format(Locale.US, "%.0f RPM", rpm))
            telemetry.addData(
                "Measured L/R",
                String.format(Locale.US, "%.0f / %.0f RPM", flywheel.measuredLeftRpm(), flywheel.measuredRightRpm()),
            )
            telemetry.addData("AT SPEED", flywheel.isAtSpeed(rpm))
            telemetry.addData(
                "Step", "%d/%d (fine %+.0f)".format(stepIdx + 1, rpmSteps.size, fineRpm),
            )
            telemetry.addLine(
                "FILM THIS SHOT, then note: " +
                    String.format(Locale.US, "%.0f", rpm) + " RPM -> ? cm",
            )
            telemetry.update()

            lastDpadUp = gamepad1.dpad_up
            lastDpadDown = gamepad1.dpad_down
            lastDpadLeft = gamepad1.dpad_left
            lastDpadRight = gamepad1.dpad_right
            lastA = gamepad1.a
            lastB = gamepad1.b
            lastX = gamepad1.x
        }

        flywheel.stop(breakMotor = true)
        intake.stop()
    }
}
