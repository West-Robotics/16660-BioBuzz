package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoControllerEx
import com.qualcomm.robotcore.hardware.ServoController
import com.qualcomm.robotcore.hardware.Servo



class DeathMetal(val hardwareMap: HardwareMap){



    val left = hardwareMap.get ("left") as DcMotorEx
    val right = hardwareMap.get ("right") as DcMotorEx


    fun init(){
        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.REVERSE
        left.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        right.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun spin(spin:Double){
        left.power = spin
        right.power = spin
    }


    fun wings(){

    }







}

