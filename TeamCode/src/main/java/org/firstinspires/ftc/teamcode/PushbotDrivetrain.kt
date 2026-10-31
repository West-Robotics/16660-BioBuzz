package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import kotlin.math.max

// ---------------------------------------------------------------
// SINGLE source of truth for the pushbot chassis.
// If the "Wheel Direction Test" shows a wheel spinning backward or
// the wrong wheel spinning, flip ONLY that wheel's value here.
// ---------------------------------------------------------------

// Motor config names on the Control Hub (must match Robot Config
// exactly, including letter case)
const val PUSHBOT_FL_NAME = "frontleft"
const val PUSHBOT_FR_NAME = "frontright"
const val PUSHBOT_BL_NAME = "backleft"
const val PUSHBOT_BR_NAME = "backright"

// Direction each motor is commanded (standard mecanum = left FWD, right REV)
val PUSHBOT_FL_DIR = DcMotorSimple.Direction.FORWARD
val PUSHBOT_FR_DIR = DcMotorSimple.Direction.REVERSE
val PUSHBOT_BL_DIR = DcMotorSimple.Direction.FORWARD
val PUSHBOT_BR_DIR = DcMotorSimple.Direction.REVERSE

// false = raw power (RUN_WITHOUT_ENCODER): robot keeps driving even if
//         an encoder wire comes loose
// true  = velocity PID (RUN_USING_ENCODER): a dead/loose encoder can
//         FREEZE that motor, so enable only after encoders pass the test
const val PUSHBOT_USE_ENCODERS = false

class PushbotDrivetrain(hardwareMap: HardwareMap) {

    val fl = hardwareMap.get(DcMotorEx::class.java, PUSHBOT_FL_NAME)
    val fr = hardwareMap.get(DcMotorEx::class.java, PUSHBOT_FR_NAME)
    val bl = hardwareMap.get(DcMotorEx::class.java, PUSHBOT_BL_NAME)
    val br = hardwareMap.get(DcMotorEx::class.java, PUSHBOT_BR_NAME)

    init {
        fl.direction = PUSHBOT_FL_DIR
        fr.direction = PUSHBOT_FR_DIR
        bl.direction = PUSHBOT_BL_DIR
        br.direction = PUSHBOT_BR_DIR

        val mode = if (PUSHBOT_USE_ENCODERS) {
            DcMotor.RunMode.RUN_USING_ENCODER
        } else {
            DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        listOf(fl, fr, bl, br).forEach {
            it.mode = mode
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    // Identical mixing to the proven TestTeleOP on the JJ branch:
    //   FL=(y+x+t)  BL=(y-x+t)  FR=(y-x-t)  BR=(y+x-t), normalized
    fun drive(forward: Double, strafe: Double, turn: Double) {
        val denom = max(abs(forward) + abs(strafe) + abs(turn), 1.0)
        fl.power = (forward + strafe + turn) / denom
        bl.power = (forward - strafe + turn) / denom
        fr.power = (forward - strafe - turn) / denom
        br.power = (forward + strafe - turn) / denom
    }

    fun stop() {
        fl.power = 0.0
        fr.power = 0.0
        bl.power = 0.0
        br.power = 0.0
    }
}
