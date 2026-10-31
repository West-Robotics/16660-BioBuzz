package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

@Autonomous(name = "Reenter Starting Area Auto", group = "Pushbot")
class ReenterStartingAreaAuto : LinearOpMode() {

    private val timer = ElapsedTime()

    // Tune these after measuring actual distance
    private val FORWARD_DISTANCE_CM = 50.0
    private val DRIVE_SPEED = 0.5
    private val CM_PER_SECOND = 60.0  // Measure and tune this

    override fun runOpMode() {
        val drive = PushbotDrivetrain(hardwareMap)

        telemetry.addLine("Reenter Starting Area Auto")
        telemetry.addLine("Press START: forward 50cm -> back 50cm")
        telemetry.update()

        waitForStart()

        // Forward 50cm
        driveStraight(drive, FORWARD_DISTANCE_CM, DRIVE_SPEED)
        sleep(300)

        // Backward 50cm (re-enter starting area)
        driveStraight(drive, -FORWARD_DISTANCE_CM, DRIVE_SPEED)

        drive.stop()
        telemetry.addLine("Done - back in starting area")
        telemetry.update()
    }

    private fun driveStraight(drive: PushbotDrivetrain, distanceCm: Double, speed: Double) {
        val targetTimeSec = abs(distanceCm) / CM_PER_SECOND
        val power = if (distanceCm >= 0) speed else -speed

        timer.reset()
        while (opModeIsActive() && timer.seconds() < targetTimeSec) {
            drive.drive(power, 0.0, 0.0)
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), targetTimeSec)
            telemetry.update()
        }
        drive.stop()
    }
}
