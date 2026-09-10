package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max
@Suppress("unused")
@TeleOp(name = "DMDrivetrain", group = "LinearOpMode")
    class DMDrivetrain : LinearOpMode() {

    override fun runOpMode() {
            val intake = hardwareMap.get("intake") as DcMotorEx

            intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            intake.direction = DcMotorSimple.Direction.FORWARD
            var ismotoron = false
            fun intaketoggle(intake: DcMotor) {
                ismotoron = !ismotoron
                val intakepower = if (ismotoron) 1.0 else 0.0
                intake.power = intakepower

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
            backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            backRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

            frontLeft.direction = DcMotorSimple.Direction.FORWARD
            frontRight.direction = DcMotorSimple.Direction.FORWARD
            backLeft.direction = DcMotorSimple.Direction.REVERSE
            backRight.direction = DcMotorSimple.Direction.REVERSE
            intake.direction = DcMotorSimple.Direction.FORWARD


            waitForStart()
            while (opModeIsActive()) {
                val x = gamepad1.left_stick_x
                val y = gamepad1.left_stick_y
                val bx = (gamepad1.right_trigger - gamepad1.right_trigger)

                val denominator = max((abs(x) + abs(y) + abs(bx)).toDouble(), 1.0)
                frontLeft.power = (y + x + bx) / denominator
                backLeft.power = (y - x + bx) / denominator
                frontRight.power = (y - x - bx) / denominator
                backRight.power = (y + x - bx) / denominator

                if (gamepad1.rightBumperWasPressed()) {
                    intaketoggle(intake)
                }

            }
        }
    }