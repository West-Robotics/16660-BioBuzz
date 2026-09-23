package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem

/**
 * ============ VISION-ONLY TEST (does NOT command the drivetrain) ============
 *
 * Field test procedure (run in order; all corrections remain disabled — this
 * OpMode constructs no drivetrain):
 *
 *  TEST 1 - Stationary robot:  features detected (blue dots turn green),
 *            stable features stay green, confidence rises, visual dx/dy/dh ~ 0.
 *  TEST 2 - Slow rotation:     static features sweep coherently; RANSAC finds
 *            the dominant rotation; outliers (RED) get rejected; dHeading
 *            matches the rotation direction/sign — else fix headingSign.
 *  TEST 3 - Translation:        push the robot forward/sideways; visual dx/dy
 *            should read +x forward (calibrate imageToRobotYawDeg until it does).
 *  TEST 4 - Dynamic obstruction: walk through the view; RED rejections dominate
 *            on the person, GREEN static features remain, confidence holds.
 *  TEST 5 - One camera blocked:  blocked cam goes stale (drops from active
 *            list), the other continues, overall confidence decreases but
 *            tracking continues.
 *  TEST 6 - Both cameras blocked: state -> LOST, confidence -> ~0, no visual
 *            motion; Road Runner would continue normally (this OpMode only
 *            proves vision output goes silent).
 *
 * Only after ALL tests pass: enable corrections in VisionAutonomous.
 */
@TeleOp(name = "Visual Localization Test", group = "Test")
class VisualLocalizationTest : LinearOpMode() {

    override fun runOpMode() {
        val vision = VisionSubsystem(hardwareMap)

        telemetry.addLine("Vision test initialized. Watch the camera previews:")
        telemetry.addLine("GREEN = static inlier, RED = outlier/moving,")
        telemetry.addLine("BLUE = new feature, YELLOW = persistent landmark.")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            vision.updateRoadRunnerPose(com.acmerobotics.roadrunner.Pose2d(0.0, 0.0, 0.0))
            vision.addTelemetry(telemetry)

            val s = vision.snapshot()
            val snapshotAgeMs = if (s != null) (System.nanoTime() - s.timestampNanos) / 1e6 else 0.0
            telemetry.addData("Snapshot age ms", "%.1f".format(snapshotAgeMs))
            telemetry.update()
            sleep(50)
        }

        vision.close()
    }
}
