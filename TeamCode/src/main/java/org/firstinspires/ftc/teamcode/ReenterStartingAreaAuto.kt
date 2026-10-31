package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime

@Autonomous(name = "Reenter Starting Area Auto", group = "Pushbot")
class ReenterStartingAreaAuto : LinearOpMode() {

    private lateinit var fl: DcMotorEx
    private lateinit var fr: DcMotorEx
    private lateinit var bl: DcMotorEx
    private lateinit var br: DcMotorEx

    private val timer = ElapsedTime()

    // Tune these after measuring actual distance
    private val FORWARD_DISTANCE_CM = 50.0
    private val DRIVE_SPEED = 0.5
    private val CM_PER_SECOND = 60.0  // Measure and tune this

    override fun runOpMode() {
        // --- Hardware ---
        fl = hardwareMap.get(DcMotorEx::class.java, "frontleft")
        fr = hardwareMap.get(DcMotorEx::class.java, "frontright")
        bl = hardwareMap.get(DcMotorEx::class.java, "backleft")
        br = hardwareMap.get(DcMotorEx::class.java, "backright")

        // Encoder mode for velocity control
        listOf(fl, fr, bl, br).forEach {
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        // Same directions as TeleOp - match the working configuration
        fl.direction = DcMotorSimple.Direction.FORWARD
        fr.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE   // Match TeleOp
        br.direction = DcMotorSimple.Direction.FORWARD  // Match TeleOp

        telemetry.addLine("Reenter Starting Area Auto")
        telemetry.addLine("Press START: forward 50cm -> back 50cm")
        telemetry.update()

        waitForStart()

        // Forward 50cm
        driveStraight(FORWARD_DISTANCE_CM, DRIVE_SPEED)
        sleep(300)

        // Backward 50cm (re-enter starting area)
        driveStraight(-FORWARD_DISTANCE_CM, DRIVE_SPEED)

        stopMotors()
        telemetry.addLine("Done - back in starting area")
        telemetry.update()
    }

    private fun driveStraight(distanceCm: Double, speed: Double) {
        val targetTimeSec = kotlin.math.abs(distanceCm) / CM_PER_SECOND
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