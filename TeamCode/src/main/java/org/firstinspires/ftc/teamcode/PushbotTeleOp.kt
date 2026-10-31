package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs

@TeleOp(name = "Pushbot TeleOp", group = "Pushbot")
class PushbotTeleOp : LinearOpMode() {

    override fun runOpMode() {
        val drive = PushbotDrivetrain(hardwareMap)

        telemetry.addLine("Pushbot TeleOp Ready")
        telemetry.addLine("Left stick: forward/strafe | Right stick X: turn")
        telemetry.addLine("Tip: run 'Wheel Direction Test' first if motion looks wrong")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            // Stick UP = forward (FTC standard: -left_stick_y, matches TestTeleOP)
            val y = -gamepad1.left_stick_y.toDouble()
            val x = gamepad1.left_stick_x.toDouble()
            val t = gamepad1.right_stick_x.toDouble()

            val deadzone = 0.08
            val f = if (abs(y) > deadzone) y else 0.0
            val s = if (abs(x) > deadzone) x else 0.0
            val turn = if (abs(t) > deadzone) t else 0.0

            drive.drive(f, s, turn)

            telemetry.addData(
                "Powers",
                "FL%+.2f FR%+.2f BL%+.2f BR%+.2f",
                drive.fl.power,
                drive.fr.power,
                drive.bl.power,
                drive.br.power,
            )
            telemetry.update()
        }

        drive.stop()
    }
}
