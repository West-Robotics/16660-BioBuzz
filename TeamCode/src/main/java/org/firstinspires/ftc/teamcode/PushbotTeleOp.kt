package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.ranges.coerceIn

@TeleOp(name = "Pushbot TeleOp", group = "Pushbot")
class PushbotTeleOp : LinearOpMode() {

    private lateinit var fl: DcMotorEx
    private lateinit var fr: DcMotorEx
    private lateinit var bl: DcMotorEx
    private lateinit var br: DcMotorEx

    override fun runOpMode() {
        // --- Hardware (names must match Robot Config) ---
        fl = hardwareMap.get(DcMotorEx::class.java, "frontleft")
        fr = hardwareMap.get(DcMotorEx::class.java, "frontright")
        bl = hardwareMap.get(DcMotorEx::class.java, "backleft")
        br = hardwareMap.get(DcMotorEx::class.java, "backright")

        // Motor directions - VERIFY WITH SPIN TEST
        // Standard: FL/FR forward, BL/BR reverse for mecanum
        // If back motors are reversed, swap BL/BR directions below
        fl.direction = DcMotorSimple.Direction.FORWARD
        fr.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE   // Try REVERSE for back motors
        br.direction = DcMotorSimple.Direction.FORWARD  // Try FORWARD for back motors

        // Encoder mode (velocity PID)
        listOf(fl, fr, bl, br).forEach {
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        telemetry.addLine("Pushbot TeleOp Ready")
        telemetry.addLine("Left stick: drive/strafe | Right stick X: turn")
        telemetry.addLine("If robot rotates instead of strafe, swap BL/BR directions")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            // --- Gamepad input ---
            val forward = gamepad1.left_stick_y.toDouble()  // forward = positive
            val strafe  = gamepad1.left_stick_x.toDouble()
            val turn    = gamepad1.right_stick_x.toDouble()

            // Deadzone
            val deadzone = 0.08
            val f = if (kotlin.math.abs(forward) > deadzone) forward else 0.0
            val s = if (kotlin.math.abs(strafe) > deadzone) strafe else 0.0
            val t = if (kotlin.math.abs(turn) > deadzone) turn else 0.0

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