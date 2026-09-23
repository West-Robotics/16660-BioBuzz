package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.opencv.core.Scalar
import java.util.Locale

@TeleOp(name = "Vision Test OpMode", group = "Test")
class VisionTestOpMode : LinearOpMode() {

    private enum class Target { POLLEN, NECTAR_BLUE, NECTAR_RED, FLOWER }
    private var currentTarget = Target.POLLEN
    
    private var hMin = 0.0
    private var hMax = 180.0
    private var sMin = 100.0
    private var sMax = 255.0
    private var vMin = 100.0
    private var vMax = 255.0

    private var selectedParam = 0 // 0: Hmin, 1: Hmax, 2: Smin, 3: Smax, 4: Vmin, 5: Vmax
    private var calibrationEnabled = false

    override fun runOpMode() {
        val visionSystem = VisionSystem(hardwareMap)

        telemetry.addData("Status", "Initialized")
        telemetry.addLine("A: Toggle Calibration Mode")
        telemetry.addLine("DPAD Up/Down: Select Parameter")
        telemetry.addLine("DPAD Left/Right: Tweak Value (X for 10x)")
        telemetry.addLine("Bumpers: Switch Target Piece")
        telemetry.update()

        // Sync initial values
        loadTargetDefaults(currentTarget)

        waitForStart()

        while (opModeIsActive()) {
            // Target Selection
            if (gamepad1.left_bumper) {
                val values = Target.entries
                currentTarget = values[(currentTarget.ordinal - 1 + values.size) % values.size]
                loadTargetDefaults(currentTarget)
                sleep(200)
            } else if (gamepad1.right_bumper) {
                val values = Target.entries
                currentTarget = values[(currentTarget.ordinal + 1) % values.size]
                loadTargetDefaults(currentTarget)
                sleep(200)
            }

            // Parameter Selection
            if (gamepad1.dpad_up) {
                selectedParam = (selectedParam - 1 + 6) % 6
                sleep(200)
            } else if (gamepad1.dpad_down) {
                selectedParam = (selectedParam + 1) % 6
                sleep(200)
            }

            // Tweak Value
            val delta = if (gamepad1.x) 10.0 else 1.0
            if (gamepad1.dpad_left) {
                updateParam(-delta)
            } else if (gamepad1.dpad_right) {
                updateParam(delta)
            }

            // Calibration Mode Toggle
            if (gamepad1.a) {
                calibrationEnabled = !calibrationEnabled
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.calibrationProcessor, calibrationEnabled)
                
                // Disable production locators when calibrating
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.pollenLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.nectarBlueLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.nectarRedLocator, !calibrationEnabled)
                visionSystem.groundPortal.setProcessorEnabled(visionSystem.flowerLocator, !calibrationEnabled)
                sleep(200)
            }

            // Update Calibration Processor
            if (calibrationEnabled) {
                visionSystem.calibrationProcessor.min = Scalar(hMin, sMin, vMin)
                visionSystem.calibrationProcessor.max = Scalar(hMax, sMax, vMax)
            }

            val detections = visionSystem.getAllDetections()
            
            telemetry.addData("Mode", if (calibrationEnabled) "CALIBRATION (CYAN BOXES)" else "DETECTION")
            telemetry.addData("Targeting", currentTarget.name)
            telemetry.addLine("--- Tweak Values ---")
            telemetry.addLine("${if (selectedParam == 0) "> " else "  "}H Min: ${hMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 1) "> " else "  "}H Max: ${hMax.toInt()}")
            telemetry.addLine("${if (selectedParam == 2) "> " else "  "}S Min: ${sMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 3) "> " else "  "}S Max: ${sMax.toInt()}")
            telemetry.addLine("${if (selectedParam == 4) "> " else "  "}V Min: ${vMin.toInt()}")
            telemetry.addLine("${if (selectedParam == 5) "> " else "  "}V Max: ${vMax.toInt()}")
            telemetry.addLine("-----------------------------")

            for (detection in detections) {
                telemetry.addLine("Item: ${detection.label}")
                telemetry.addLine(String.format(Locale.US, "  Distance: %.1f mm", detection.distanceMm))
                telemetry.addLine(String.format(Locale.US, "  Angle: %.1f deg", detection.angleDegrees))
            }

            telemetry.update()
            sleep(50)
        }

        visionSystem.close()
    }

    private fun loadTargetDefaults(target: Target) {
        // Since ColorRange.min/max are protected, we use the known defaults from globals.kt
        when(target) {
            Target.POLLEN -> {
                hMin = 10.0; hMax = 40.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.NECTAR_BLUE -> {
                hMin = 100.0; hMax = 140.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.NECTAR_RED -> {
                hMin = 0.0; hMax = 10.0; sMin = 100.0; sMax = 255.0; vMin = 100.0; vMax = 255.0
            }
            Target.FLOWER -> {
                hMin = 40.0; hMax = 80.0; sMin = 50.0; sMax = 255.0; vMin = 50.0; vMax = 255.0
            }
        }
    }

    private fun updateParam(delta: Double) {
        when(selectedParam) {
            0 -> hMin = (hMin + delta).coerceIn(0.0, 180.0)
            1 -> hMax = (hMax + delta).coerceIn(0.0, 180.0)
            2 -> sMin = (sMin + delta).coerceIn(0.0, 255.0)
            3 -> sMax = (sMax + delta).coerceIn(0.0, 255.0)
            4 -> vMin = (vMin + delta).coerceIn(0.0, 255.0)
            5 -> vMax = (vMax + delta).coerceIn(0.0, 255.0)
        }
    }
}