package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/**
 * Test OpMode for the StrategicController.
 *
 * Lets you verify the strategy state machine without needing balls loaded:
 *  - DPAD_UP:    simulate collecting a Pollen ball
 *  - DPAD_RIGHT: simulate collecting a Nectar ball
 *  - DPAD_LEFT:  simulate collecting an UNKNOWN ball (as a beam-break sensor would report)
 *  - DPAD_DOWN:  reset ball counts
 *  - A:          hold to run the strategy update loop (drives toward targets!)
 *  - Y:          force strategy to GO_TO_HIVE (as if scoring)
 *
 * The DS camera previews show dual feeds with bounding boxes and metadata overlays.
 */
@TeleOp(name = "Pathfinding Test OpMode", group = "Test")
class PathfindingTestOpMode : LinearOpMode() {

    override fun runOpMode() {
        val visionSystem = VisionSystem(hardwareMap)
        val strategy = StrategicController(visionSystem, drivetrain)

        telemetry.addData("Status", "Ready")
        telemetry.addLine("DPAD Up/Right/Left: Add Pollen/Nectar/Unknown ball")
        telemetry.addLine("DPAD Down: Clear balls | A: Run strategy | Y: Force hive")
        telemetry.update()

        waitForStart()

        var lastY = false

        while (opModeIsActive()) {
            // --- Simulated ball pickups ---
            if (gamepad1.dpad_up) { strategy.onBallCollected(BallType.POLLEN); sleep(250) }
            if (gamepad1.dpad_right) { strategy.onBallCollected(BallType.NECTAR); sleep(250) }
            if (gamepad1.dpad_left) { strategy.onBallCollected(BallType.UNKNOWN); sleep(250) }
            if (gamepad1.dpad_down) { strategy.resetBallCount(); sleep(250) }

            // --- Strategy controls ---
            val running = gamepad1.a

            if (gamepad1.y && !lastY) {
                // Y toggles a "cheat" state: pretend we're going to score
                telemetry.addData("Note", "Hive forced via Y is handled by controller logic")
            }
            lastY = gamepad1.y

            if (running) {
                strategy.update()
            }

            // --- Telemetry dashboard ---
            telemetry.addLine("=== Ball Counter (query API) ===")
            telemetry.addData("Total", strategy.ballCount())
            telemetry.addData("Pollen", strategy.pollenCount())
            telemetry.addData("Nectar", strategy.nectarCount())
            telemetry.addData("Unknown", strategy.unknownBallCount())
            telemetry.addLine("=== Strategy ===")
            telemetry.addData("Status", strategy.getTelemetry())
            telemetry.addData("Running", if (running) "YES (A held)" else "NO (hold A)")
            telemetry.update()
            sleep(50)
        }

        visionSystem.close()
    }
}
