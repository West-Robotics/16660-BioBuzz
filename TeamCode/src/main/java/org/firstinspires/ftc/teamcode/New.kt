package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

class New: LinearOpMode() {
    override fun runOpMode() {
        val motor1 = hardwareMap.get("name") as DcMotorEx
        motor1.direction = DcMotorSimple.Direction.REVERSE
        motor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val servo1 = hardwareMap.get("servo1") as Servo


        waitForStart()
        while (opModeIsActive()){

            if (gamepad1.a){
                motor1.power = 0.1
            }
            if (gamepad1.right_bumper){
                servo1.position = 1.0
            }



        }



    }
}