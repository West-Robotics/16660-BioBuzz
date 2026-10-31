package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime

@Autonomous(name = "Reenter Starting Area Auto", group = "Pushbot")
class ReenterStartingAreaAuto : LinearOpMode() {

    private lateinit var fl: DcMotor
    private lateinit var fr: DcMotor
    private lateinit var bl: DcMotor
    private lateinit var br: DcMotor

    private val timer = ElapsedTime()

    // Tunable - adjust after measuring actual distance
    private val FORWARD_DISTANCE_CM = 50.0
    private val DRIVE_SPEED = 0.4
    private val CM_PER_SECOND = 50.0  // Measure and tune this

    override fun runOpMode() {
        // --- Hardware (names must match Robot Config) ---
        fl = hardwareMap.get(DcMotor::class.java, "frontleft")
        fr = hardwareMap.get(DcMotor::class.java, "frontright")
        bl = hardwareMap.get(DcMotor::class.java, "backleft")
        br = hardwareMap.get(DcMotor::class.java, "backright")

        // Motor directions - VERIFY WITH SPIN TEST
        fl.direction = DcMotorSimple.Direction.FORWARD
        fr.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.FORWARD
        br.direction = DcMotorSimple.Direction.REVERSE

        // Raw power mode (no encoders)
        listOf(fl, fr, bl, br).forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        telemetry.addLine("Reenter Starting Area Auto")
        telemetry.addLine("Press START to begin")
        telemetry.update()

        waitForStart()

        // --- Step 1: Drive forward 50 cm ---
        driveStraight(FORWARD_DISTANCE_CM, DRIVE_SPEED)

        // --- Step 2: Brief pause ---
        sleep(300)

        // --- Step 3: Drive backward 50 cm (re-enter starting area) ---
        driveStraight(-FORWARD_DISTANCE_CM, DRIVE_SPEED)

        // Stop
        stopMotors()
        telemetry.addLine("Done - back in starting area")
        telemetry.update()
    }

    private fun driveStraight(distanceCm: Double, speed: Double) {
        val targetTimeSec = abs(distanceCm) / CM_PER_SECOND
        val power = if (distanceCm >= 0) speed else -speed

        timer.reset()
        while (opModeIsActive() && timer.seconds() < targetTimeSec) {
            fl.power = power
            fr.power = power
            bl.power = power
            br.power = power
            telemetry.addData("Time", "%.1f / %.1f s", timer.seconds(), targetTimeSec)
            telemetry.update()
        }
        stopMotors()
    }

    private fun stopMotors() {
        listOf(fl, fr, bl, br).forEach { it.power = 0.0 }
    }
}