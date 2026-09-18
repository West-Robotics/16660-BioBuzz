package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max

@TeleOp(name = "Test TeleOp")
class TestTeleOP : LinearOpMode(){
    override fun runOpMode() {
        val frontLeft = hardwareMap.get("frontLeft") as DcMotorEx
        val frontRight = hardwareMap.get("frontRight") as DcMotorEx
        val backLeft = hardwareMap.get("backLeft") as DcMotorEx
        val backRight = hardwareMap.get("backRight") as DcMotorEx

        val motors = mutableListOf<DcMotorEx>(frontRight, frontLeft, backRight, backLeft)

        for (motor in motors){
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        frontLeft.direction = DcMotorSimple.Direction.FORWARD
        backLeft.direction = DcMotorSimple.Direction.FORWARD
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE


        waitForStart()
        while (opModeIsActive()){
            val x = gamepad1.left_stick_x.toDouble()
            val y = -gamepad1.left_stick_y.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()


            val denominator = max(abs(x)+abs(y)+abs(rx),1.0)
            frontLeft.power = (y+x+rx)/denominator
            backLeft.power = (y-x+rx)/denominator
            frontRight.power = (y-x-rx)/denominator
            backRight.power = (y+x-rx)/denominator


        }
    }
}