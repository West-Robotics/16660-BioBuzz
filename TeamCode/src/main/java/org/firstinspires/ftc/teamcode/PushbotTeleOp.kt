package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "Pushbot TeleOp", group = "Pushbot")
class PushbotTeleOp : LinearOpMode() {

    private lateinit var fl: DcMotor
    private lateinit var fr: DcMotor
    private lateinit var bl: DcMotor
    private lateinit var br: DcMotor

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

        telemetry.addLine("Pushbot TeleOp Ready")
        telemetry.addLine("Left stick: drive/strafe | Right stick X: turn")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            // --- Gamepad input ---
            val forward = -gamepad1.left_stick_y
            val strafe  = gamepad1.left_stick_x
            val turn    = gamepad1.right_stick_x

            // Deadzone
            val deadzone = 0.08
            val f = if (abs(forward) > deadzone) forward else 0.0
            val s = if (abs(strafe) > deadzone) strafe else 0.0
            val t = if (abs(turn) > deadzone) turn else 0.0

            // --- Mecanum mixing ---
            val flPower = (f + s + t).coerceIn(-1.0, 1.0)
            val frPower = (f - s - t).coerceIn(-1.0, 1.0)
            val blPower = (f - s + t).coerceIn(-1.0, 1.0)
            val brPower = (f + s - t).coerceIn(-1.0, 1.0)

            fl.power = flPower
            fr.power = frPower
            bl.power = blPower
            br.power = brPower

            telemetry.addData("Powers", "%.2f %.2f %.2f %.2f", flPower, frPower, blPower, brPower)
            telemetry.update()
        }

        // Stop on exit
        listOf(fl, fr, bl, br).forEach { it.power = 0.0 }
    }
}