package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx

class Raziel: LinearOpMode() {

    // DATA TYPES
    //INT  10 12 13
    //FLOAT  11.1111 12.2222
    //DOUBLE  11.111111111 12.22222222
    //BOOLEAN  true false
    //STRING "HI" "Hello World"
    val motor = hardwareMap.get("name")

    override fun runOpMode(){
        val frontLeft = hardwareMap.get("motorname") as DcMotorEx


    }

}