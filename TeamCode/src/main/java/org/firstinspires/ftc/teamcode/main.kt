package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.ServoControllerEx
import com.qualcomm.robotcore.hardware.Servo


class Experiment : LinearOpMode() {
    override fun runOpMode() {

        val scoopx = gamepad1.right_stick_x
        val scoop1 = hardwareMap.get("scoop1") as Servo
        val scoop2 = hardwareMap.get("scoop2") as Servo
        if (scoopx < 0)
            scoop2.position


    }




}
