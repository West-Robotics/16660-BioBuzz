package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1
import com.qualcomm.robotcore.hardware.Gamepad


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

