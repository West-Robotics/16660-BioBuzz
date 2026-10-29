package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max

@TeleOp(name = "DMDrivetrain")
class DeathMetalDrivetrain : LinearOpMode() {

    override fun runOpMode() {

        val spinner = DeathMetal(hardwareMap)

        spinner.init()

        val intake = hardwareMap.get("intake") as DcMotorEx
        val left = hardwareMap.get("left") as DcMotorEx
        val right = hardwareMap.get("right") as DcMotorEx
        val leftfeeder = hardwareMap.get("leftfeeder") as DcMotorEx
        val rightfeeder = hardwareMap.get("rightfeeder") as DcMotorEx
        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.REVERSE
        left.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        right.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftfeeder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftfeeder.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        leftfeeder.direction = DcMotorSimple.Direction.FORWARD
        rightfeeder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightfeeder.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rightfeeder.direction = DcMotorSimple.Direction.REVERSE
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        intake.direction = DcMotorSimple.Direction.FORWARD

        var ismotoron = false
        fun intaketoggle(intake: DcMotor) {
            ismotoron = !ismotoron
            val intakepower = if (ismotoron) 1.0 else 0.0
            intake.power = intakepower

        }


        var feedon = false
        fun feedtoggle(rightfeeder: DcMotorEx, leftfeeder: DcMotorEx) {
            feedon = !feedon
            val feedpower = if (feedon) 0.5 else 0.0
            leftfeeder.power = feedpower
            rightfeeder.power = feedpower
        }

        var spinspin = false
        fun spintoggle(right: DcMotor, left: DcMotor) {
            spinspin = !spinspin
            val spin = if (spinspin) 0.5 else 0.0
            left.power = spin
            right.power = spin
        }


                val frontLeft = hardwareMap.get("frontLeft") as DcMotorEx
                val frontRight = hardwareMap.get("frontRight") as DcMotorEx
                val backLeft = hardwareMap.get("backLeft") as DcMotorEx
                val backRight = hardwareMap.get("backRight") as DcMotorEx



                frontLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                frontRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                backLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                backRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


                frontLeft.direction = DcMotorSimple.Direction.FORWARD
                frontRight.direction = DcMotorSimple.Direction.REVERSE
                backLeft.direction = DcMotorSimple.Direction.FORWARD
                backRight.direction = DcMotorSimple.Direction.REVERSE



                waitForStart()
                while (opModeIsActive()) {
                    val x = gamepad1.left_stick_x
                    val y = -gamepad1.left_stick_y
                    val bx = (gamepad1.right_trigger - gamepad1.left_trigger)

                    val denominator = max((abs(x) + abs(y) + abs(bx)).toDouble(), 1.0)
                    frontLeft.power = (y + x + bx) / denominator
                    backLeft.power = (y - x + bx) / denominator
                    frontRight.power = (y - x - bx) / denominator
                    backRight.power = (y + x - bx) / denominator

                    if (gamepad1.aWasPressed()) {
                        spintoggle(right, left)
                    }

                    if (gamepad1.leftBumperWasPressed()) {
                        feedtoggle(rightfeeder, leftfeeder)
                    }

                    if (gamepad1.rightBumperWasPressed()) {
                        intaketoggle(intake)
                    }

                }
            }
}
