package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.blackbox.BlackBox

/**
 * ============ HARDWARE PROBE — boot-time graceful degradation ============
 *
 * Called at the top of every OpMode. Scans the hardware CONFIGURATION (never
 * touches the devices) and reports what exists, so OpModes can fall back
 * instead of crashing:
 *
 *  * BioBuzz TeleOp: missing cameras -> "AUTONOMOUS HARDWARE NOT DETECTED -
 *    falling back to MANUAL ONLY mode" — assists/vision/shadow planner are
 *    skipped, the human keeps full manual control of whatever IS configured.
 *  * Autonomous / SelfPlay / NN Test / Calibration: missing kit -> banner +
 *    safely stay parked (never crash, never move).
 *
 * The probe result is also written to the Black Box ("hardware_probe" event)
 * so post-match analysis knows exactly what robot state each session ran in.
 */
object HardwareProbe {

    data class Report(
        val driveMotors: Boolean,
        val upCamera: Boolean,
        val frontCamera: Boolean,
        val imu: Boolean,
        val flywheelLeft: Boolean,
        val flywheelRight: Boolean,
        val intakeMotor: Boolean,
        val loaderServo: Boolean,
    ) {
        /** Both cameras present — the vision stack may be built. */
        val vision: Boolean get() = upCamera && frontCamera

        /** Both flywheel motors present. */
        val shooter: Boolean get() = flywheelLeft && flywheelRight

        /** Everything the autonomous / SelfPlay stack touches. */
        val fullAutonomy: Boolean
            get() = driveMotors && vision && imu && shooter && intakeMotor && loaderServo

        /** Human-readable list of what's absent, for the fallback banner. */
        val missingList: List<String>
            get() {
                val out = mutableListOf<String>()
                if (!driveMotors) out += "drive motors"
                if (!upCamera) out += "UP camera"
                if (!frontCamera) out += "FRONT camera"
                if (!imu) out += "imu"
                if (!flywheelLeft) out += "flywheelleft motor"
                if (!flywheelRight) out += "flywheelright motor"
                if (!intakeMotor) out += "intake motor"
                if (!loaderServo) out += "loader servo"
                return out
            }

        fun logToBlackBox() {
            BlackBox.event(
                "hardware_probe",
                "driveMotors" to driveMotors,
                "upCamera" to upCamera, "frontCamera" to frontCamera,
                "imu" to imu, "flywheelLeft" to flywheelLeft,
                "flywheelRight" to flywheelRight, "intakeMotor" to intakeMotor,
                "loaderServo" to loaderServo,
                "fullAutonomy" to fullAutonomy,
            )
        }
    }

    /** Probes the configuration via tryGet — never throws, never opens devices. */
    fun probe(hardwareMap: HardwareMap): Report {
        fun <T> has(cls: Class<T>, name: String): Boolean =
            try {
                hardwareMap.tryGet(cls, name) != null
            } catch (_: Exception) {
                false
            }

        val driveMotors = has(DcMotor::class.java, DrivetrainFrontLeft) &&
            has(DcMotor::class.java, DrivetrainFrontRight) &&
            has(DcMotor::class.java, DrivetrainBackLeft) &&
            has(DcMotor::class.java, DrivetrainBackRight)

        return Report(
            driveMotors = driveMotors,
            upCamera = has(WebcamName::class.java, UP_CAM_NAME),
            frontCamera = has(WebcamName::class.java, FRONT_CAM_NAME),
            imu = has(IMU::class.java, RR_IMU_NAME),
            flywheelLeft = has(DcMotor::class.java, FlywheelLeftMotor),
            flywheelRight = has(DcMotor::class.java, FlywheelRightMotor),
            intakeMotor = has(DcMotor::class.java, IntakeMotor),
            loaderServo = has(Servo::class.java, LoaderServoName),
        )
    }
}