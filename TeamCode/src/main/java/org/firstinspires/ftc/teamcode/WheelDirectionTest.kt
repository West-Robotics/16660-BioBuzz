package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

// 30-second hardware truth test:
//   Hold A = frontleft      Hold B = frontright
//   Hold X = backleft       Hold Y = backright
//   D-pad UP = all four forward
// Expected: the named wheel rolls FORWARD (toward the front of the robot).
//   - A different wheel moves  -> port/name mapping is wrong; fix Robot Config
//     or the *_NAME values in PushbotDrivetrain.kt
//   - Wheel spins backward     -> flip that wheel's *_DIR in PushbotDrivetrain.kt
//   - Encoder number frozen while wheel spins -> encoder wire loose/disconnected
@TeleOp(name = "Wheel Direction Test", group = "Pushbot")
class WheelDirectionTest : LinearOpMode() {

    override fun runOpMode() {
        val drive = PushbotDrivetrain(hardwareMap)

        // Always raw power for this test, regardless of PUSHBOT_USE_ENCODERS
        listOf(drive.fl, drive.fr, drive.bl, drive.br).forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        val testPower = 0.4

        telemetry.addLine("A=frontleft  B=frontright  X=backleft  Y=backright")
        telemetry.addLine("D-pad UP = all four forward. Press RUN then hold buttons.")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            drive.fl.power = if (gamepad1.a) testPower else 0.0
            drive.fr.power = if (gamepad1.b) testPower else 0.0
            drive.bl.power = if (gamepad1.x) testPower else 0.0
            drive.br.power = if (gamepad1.y) testPower else 0.0
            if (gamepad1.dpad_up) {
                drive.fl.power = testPower
                drive.fr.power = testPower
                drive.bl.power = testPower
                drive.br.power = testPower
            }

            telemetry.addData(
                "frontleft",
                "dir=%s  enc=%d",
                PUSHBOT_FL_DIR,
                drive.fl.currentPosition,
            )
            telemetry.addData(
                "frontright",
                "dir=%s  enc=%d",
                PUSHBOT_FR_DIR,
                drive.fr.currentPosition,
            )
            telemetry.addData(
                "backleft",
                "dir=%s  enc=%d",
                PUSHBOT_BL_DIR,
                drive.bl.currentPosition,
            )
            telemetry.addData(
                "backright",
                "dir=%s  enc=%d",
                PUSHBOT_BR_DIR,
                drive.br.currentPosition,
            )
            telemetry.update()
        }

        drive.stop()
    }
}
