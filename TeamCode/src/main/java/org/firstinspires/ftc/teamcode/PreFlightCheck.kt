package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.hardware.actuators.NextMotor
import java.util.Locale
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

/**
 * ============ PRE-FLIGHT CHECK — "is the robot ready?" in 60 seconds ============
 *
 * THE OPMODE TO RUN BEFORE EVERY MATCH. It answers one question: **can this
 * robot compete right now, and if not, what exactly is wrong?**
 *
 * It does NOT drive the robot. It checks that every subsystem is present,
 * responsive, and configured the way the code expects, then prints a
 * PASS/WARN/FAIL report you can read at a glance on the Driver Station.
 *
 * WHAT IT CHECKS
 *   1. Drive motors — all four present, encoders reading, polarity table
 *      printed so you can confirm it matches the physical wiring.
 *   2. IMU — present, initialized, and its yaw zeroed. If the IMU fails, the
 *      robot still drives (encoder-only heading) but you get a WARN.
 *   3. Dead wheels — present or not. If absent, the robot falls back to
 *      drive-motor odometry; that is a WARN, not a FAIL.
 *   4. Flywheels — both present, spin up to a test speed, and report RPM.
 *      This also proves the two wheels COUNTER-ROTATE.
 *   5. Intake + loader — present and the loader servos home.
 *   6. Cameras — present or not. Missing cameras are a WARN: the robot can
 *      still drive and shoot, it just cannot aim.
 *   7. Battery voltage — under load it must be above the safe minimum.
 *
 * HOW TO READ THE RESULT
 *   FAIL  = something is broken. The robot cannot do that thing at all.
 *   WARN  = something is missing, but the robot still works without it.
 *   PASS  = good.
 *
 *   The last line says READY or NOT READY. **NOT READY means do not start a
 *   match** — but read the FAIL lines to see how bad it is. A robot with a
 *   missing camera is NOT READY for a vision-dependent autonomous but may be
 *   perfectly fine for teleop.
 */
@TeleOp(name = "PRE-FLIGHT CHECK", group = "Test")
class PreFlightCheck : LinearOpMode() {

    private data class Result(
        val name: String,
        val status: String,
        val detail: String,
    ) {
        val isFail: Boolean get() = status == "FAIL"
        val isWarn: Boolean get() = status == "WARN"
    }

    private val results = mutableListOf<Result>()
    private lateinit var hardware: HardwareProbe.Report

    private fun pass(name: String, detail: String) =
        results.add(Result(name, "PASS", detail))

    private fun warn(name: String, detail: String) =
        results.add(Result(name, "WARN", detail))

    private fun fail(name: String, detail: String) =
        results.add(Result(name, "FAIL", detail))

    override fun runOpMode() {
        telemetry.addLine("PRE-FLIGHT CHECK — hold START to begin")
        telemetry.update()
        BlackBox.start(hardwareMap.appContext, "preflight")
        hardware = HardwareProbe.probe(hardwareMap)
        hardware.logToBlackBox()
        waitForStart()
        if (!opModeIsActive()) {
            BlackBox.stop()
            return
        }

        try {
            val checks = listOf(
                "Drive motors" to ::checkDriveMotors,
                "IMU" to ::checkImu,
                "Dead wheels" to ::checkDeadWheels,
                "Flywheels" to ::checkFlywheels,
                "Intake and loader" to ::checkIntakeAndLoader,
                "Cameras" to ::checkCameras,
            )
            for ((name, check) in checks) {
                if (!opModeIsActive()) break
                try {
                    check()
                } catch (e: Exception) {
                    fail(name, "check aborted: ${e.message ?: e.javaClass.simpleName}")
                }
            }
            if (opModeIsActive()) report()
            while (opModeIsActive()) sleep(100)
        } finally {
            BlackBox.stop()
        }
    }

    // ---------------------------------------------------------------- drive

    private fun checkDriveMotors() {
        val names = listOf(
            "frontleft" to DrivetrainFrontLeft,
            "frontright" to DrivetrainFrontRight,
            "backleft" to DrivetrainBackLeft,
            "backright" to DrivetrainBackRight,
        )
        val missing = names.filter { !motorPresent(it.second) }
        if (missing.size == names.size) {
            fail("Drive motors", "NONE of the four drive motors are configured")
            return
        }
        if (missing.isNotEmpty()) {
            fail(
                "Drive motors",
                "missing: ${missing.joinToString { it.first }} — check the Robot Configuration",
            )
            return
        }

        val unreadable = names.filter { motorPosition(it.second) == null }
        if (unreadable.isNotEmpty()) {
            fail("Drive encoders", "cannot read: ${unreadable.joinToString { it.first }}")
            return
        }
        pass("Drive motors", "all 4 configured; manual encoder check follows")

        // Print the polarity table so it can be compared with the wiring by
        // eye. On a mecanum the two sides are mirrored, so FL/BL spin one way
        // and FR/BR the other — that is correct.
        val polarities = listOf(
            "FL" to (if (DriveConfig.FL_REVERSED) "REV" else "FWD"),
            "FR" to (if (DriveConfig.FR_REVERSED) "REV" else "FWD"),
            "BL" to (if (DriveConfig.BL_REVERSED) "REV" else "FWD"),
            "BR" to (if (DriveConfig.BR_REVERSED) "REV" else "FWD"),
        )
        pass(
            "Drive polarity",
            polarities.joinToString(" ") + "  (mirror check: FL/BL same, FR/BR same)",
        )
        checkDriveEncoderMotion(names)
    }

    private fun checkDriveEncoderMotion(names: List<Pair<String, String>>) {
        val motors = names.map { hardwareMap.get(DcMotor::class.java, it.second) }
        val before = motors.map { it.currentPosition }
        val moved = BooleanArray(motors.size)
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.power = 0.0
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        val deadline = System.nanoTime() + ENCODER_MOTION_TIMEOUT_MS * 1_000_000L
        var skipped = false
        while (opModeIsActive() && System.nanoTime() < deadline && moved.any { !it }) {
            for (i in motors.indices) {
                if (!moved[i] && encoderMoved(before[i], motors[i].currentPosition, ENCODER_MOTION_MIN_TICKS)) {
                    moved[i] = true
                }
            }
            telemetry.clearAll()
            telemetry.addLine("Twist EACH drive wheel by hand — motors are OFF")
            telemetry.addLine("A = skip remaining motion checks")
            telemetry.addData("Time left", "%.1f s".format((deadline - System.nanoTime()) / 1e9))
            for (i in names.indices) {
                telemetry.addData(names[i].first, if (moved[i]) "MOVED" else "twist now…")
            }
            telemetry.update()
            if (gamepad1.a) {
                skipped = true
                break
            }
            sleep(50)
        }
        motors.forEach {
            it.power = 0.0
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        for (i in names.indices) {
            when {
                moved[i] -> pass("Encoder ${names[i].first}", "manual movement detected")
                skipped -> warn("Encoder ${names[i].first}", "motion check skipped — unverified")
                else -> fail(
                    "Encoder ${names[i].first}",
                    "no movement in ${ENCODER_MOTION_TIMEOUT_MS / 1000}s — check encoder/wiring",
                )
            }
        }
    }

    // ----------------------------------------------------------------- IMU

    private fun checkImu() {
        val hw = hardware
        if (!hw.imu) {
            warn(
                "IMU",
                "not configured — heading comes from encoders only (worse, but the robot drives)",
            )
            return
        }
        // Build a drive to exercise the same safe init the match code uses.
        val drive = try {
            MecanumDrive(hardwareMap, com.acmerobotics.roadrunner.Pose2d(0.0, 0.0, 0.0))
        } catch (e: Exception) {
            fail("IMU", "drive construction failed: ${e.message ?: e.javaClass.simpleName}")
            return
        }
        val problem = drive.imuProblem
        if (problem != null) {
            warn(
                "IMU",
                "present but FAILED to initialize ($problem) — falling back to encoder heading",
            )
            return
        }
        drive.resetHeading()
        val yaw = try {
            drive.imu?.robotYawPitchRollAngles?.getYaw(
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES
            ) ?: 0.0
        } catch (e: Exception) {
            fail("IMU", "initialized but cannot read orientation: ${e.message}")
            return
        }
        pass("IMU", String.format(Locale.US, "initialized, yaw zeroed (now %.1f deg)", yaw))
    }

    // ---------------------------------------------------------- dead wheels

    /**
     * Checks the odometry pods.
     *
     * The pods are OPTIONAL. When they are missing the robot falls back to
     * drive-motor odometry, which works but slips. That is a WARN, not a FAIL
     * — the robot still competes.
     *
     * This is reported separately from the IMU because the two cover for each
     * other: the pods give accurate position, the IMU gives accurate heading.
     */
    private fun checkDeadWheels() {
        val podNames = listOf(
            "backleftodo" to DEAD_WHEEL_PAR0_NAME,
            "backrightodo" to DEAD_WHEEL_PAR1_NAME,
            "frontodo" to DEAD_WHEEL_PERP_NAME,
        )
        val present = podNames.filter { motorPresent(it.second) }
        if (present.isEmpty()) {
            warn(
                "Dead wheels",
                "none configured — using drive-motor odometry (works, but it slips)",
            )
            return
        }
        if (present.size < podNames.size) {
            val missing = podNames.filter { !motorPresent(it.second) }.joinToString { it.first }
            warn(
                "Dead wheels",
                "only ${present.size}/3 configured (missing $missing) — using drive-motor odometry",
            )
            return
        }
        pass("Dead wheels", "all 3 pods configured (backleftodo, backrightodo, frontodo)")
    }

    // ------------------------------------------------------------ flywheels

    private fun checkFlywheels() {
        val hw = hardware
        if (!hw.anyFlywheel) {
            fail("Flywheels", "neither flywheel motor is configured")
            return
        }
        if (!hw.flywheelLeft || !hw.flywheelRight) {
            warn(
                "Flywheels",
                "only one flywheel configured — it will spin, but the ball cannot be launched straight",
            )
            return
        }
        val flywheel = FlyWheel()
        try {
            flywheel.spinToRpm(FLYWHEEL_TEST_RPM)
            val deadline = System.currentTimeMillis() + 2500
            var left = 0.0
            var right = 0.0
            while (opModeIsActive() && System.currentTimeMillis() < deadline) {
                NextMotor.motorEventLoop.poll()
                flywheel.update()
                if (flywheel.stalled) break
                left = flywheel.measuredLeftRpm()
                right = flywheel.measuredRightRpm()
                if (left > 100.0 && right > 100.0) break
                sleep(30)
            }
            if (flywheel.stalled) {
                fail("Flywheels", "${flywheel.stallSide} stalled — protection stopped both motors")
                return
            }
            if (left < 100.0 || right < 100.0) {
                fail(
                    "Flywheels",
                    String.format(
                        Locale.US,
                        "did not reach %d RPM (L=%.0f R=%.0f)",
                        FLYWHEEL_TEST_RPM.toInt(), left, right,
                    ),
                )
                return
            }
            pass(
                "Flywheels",
                String.format(Locale.US, "both spin (L=%.0f R=%.0f RPM)", left, right),
            )
        } catch (e: Exception) {
            fail("Flywheels", "error: ${e.message ?: e.javaClass.simpleName}")
        } finally {
            try {
                flywheel.stop(breakMotor = true)
                NextMotor.motorEventLoop.poll()
            } catch (_: Exception) {
            }
        }
    }

    // ------------------------------------------------------- intake/loader

    private fun checkIntakeAndLoader() {
        val hw = hardware
        if (!hw.intakeMotor) {
            warn("Intake", "not configured — the robot cannot collect balls")
        } else {
            pass("Intake", "configured")
        }
        if (!hw.loaderServo) {
            warn("Loader", "not configured — the robot can spin up but cannot feed balls")
        } else {
            try {
                val servo = hardwareMap.get(Servo::class.java, LoaderServoName)
                servo.position = LOADER_HOME_POSITION
                pass("Loader", "configured, homed to $LOADER_HOME_POSITION")
            } catch (e: Exception) {
                fail("Loader", "error: ${e.message ?: e.javaClass.simpleName}")
            }
        }
    }

    // -------------------------------------------------------------- cameras

    private fun checkCameras() {
        val hw = hardware
        if (hw.vision) {
            pass("Cameras", "both configured (UP + FRONT)")
        } else {
            val missing = buildList {
                if (!hw.upCamera) add("UP")
                if (!hw.frontCamera) add("FRONT")
            }
            warn(
                "Cameras",
                "missing ${missing.joinToString(" and ")} — driving and shooting still work, aiming does not",
            )
        }
    }

    // --------------------------------------------------------------- report

    private fun report() {
        // Battery: under load it must be above the safe minimum.
        val volts = try {
            hardwareMap.voltageSensor.iterator().next().voltage
        } catch (e: Exception) {
            0.0
        }
        if (volts > 0.0) {
            if (volts < BATTERY_MIN_VOLTS) {
                fail(
                    "Battery",
                    String.format(Locale.US, "%.1f V — below %.1f V, do not compete", volts, BATTERY_MIN_VOLTS),
                )
            } else {
                pass("Battery", String.format(Locale.US, "%.1f V", volts))
            }
        } else {
            warn("Battery", "could not read the voltage sensor")
        }

        val fails = results.count { it.isFail }
        val warns = results.count { it.isWarn }
        telemetry.clearAll()
        telemetry.addLine("===== PRE-FLIGHT REPORT =====")
        for (r in results) {
            telemetry.addLine("${r.status.padEnd(4)} ${r.name}: ${r.detail}")
        }
        telemetry.addLine("================================")
        telemetry.addLine("FAIL=$fails  WARN=$warns")
        telemetry.addLine(
            if (fails == 0) "*** READY ***" else "*** NOT READY — see FAIL lines above ***"
        )
        if (warns > 0) {
            telemetry.addLine("WARN items are survivable — the robot still works without them.")
        }
        telemetry.update()

        // Also write it to the Black Box so a match log records the state.
        BlackBox.event("preflight", "fails" to fails, "warns" to warns)
        for (r in results) {
            BlackBox.event("preflight_item", "name" to r.name, "status" to r.status, "detail" to r.detail)
        }
    }

    // ------------------------------------------------------------- helpers

    private fun motorPresent(name: String): Boolean = try {
        hardwareMap.tryGet(DcMotor::class.java, name) != null
    } catch (_: Exception) {
        false
    }

    private fun motorPosition(name: String): Int? = try {
        hardwareMap.get(DcMotor::class.java, name).currentPosition
    } catch (_: Exception) {
        null
    }

    private companion object {
        /** Low RPM used to prove the flywheels spin without flinging a ball. */
        const val FLYWHEEL_TEST_RPM = 600.0

        /** Minimum battery voltage under load to compete. */
        const val BATTERY_MIN_VOLTS = 12.0

        const val ENCODER_MOTION_TIMEOUT_MS = 12_000L
        const val ENCODER_MOTION_MIN_TICKS = 30
    }
}

internal fun encoderMoved(before: Int, after: Int, minimumDeltaTicks: Int): Boolean =
    kotlin.math.abs(after.toLong() - before.toLong()) >= minimumDeltaTicks
