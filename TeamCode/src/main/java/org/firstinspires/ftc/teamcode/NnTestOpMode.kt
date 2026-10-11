package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.nn.DetectorProcessor
import org.firstinspires.ftc.teamcode.nn.NeuralReferee
import org.firstinspires.ftc.teamcode.nn.ShotPredictor
import org.firstinspires.ftc.teamcode.vision.VisionSubsystem
import java.util.Locale

/**
 * ============ NEURAL NET TEST — trust-building bench ============
 *
 * Runs both on-robot neural nets against the live cameras WITHOUT any robot
 * motion, so you can validate them on the bench:
 *
 *  * the ball DETECTOR's verdicts (boxes are also drawn cyan on the DS
 *    preview by the processor itself), with the current authority mode,
 *  * the SHOT PREDICTOR's P(hit) across a range of distances at the RPM the
 *    calibrated curve commands, at the live battery voltage.
 *
 * Interpretation: the predictor table should look like a sensible curve —
 * high P(hit) where the shooter is dialed in, dropping off at the extremes.
 * If it's flat or weird, the training data or model is bad; retrain.
 */
@TeleOp(name = "Neural Net Test", group = "Test")
class NnTestOpMode : LinearOpMode() {

    override fun runOpMode() {
        ShotPredictor.ensureLoaded(hardwareMap.appContext)
        val ballLocator = BallLocator()
        val detector = DetectorProcessor(hardwareMap.appContext)
        val vision = VisionSubsystem(
            hardwareMap,
            extraUpProcessors = emptyList(),
            extraFrontProcessors = ballLocator.processors + detector,
        )

        telemetry.addData("Detector", if (detector.loaded) "model loaded" else "no model file")
        telemetry.addData("Detector mode", NN_DETECTOR_MODE.name)
        telemetry.addData("Shot predictor", if (ShotPredictor.available) "loaded" else "not loaded")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            vision.periodic()

            val targets = ballLocator.visibleTargets()
            val nn = NeuralReferee.latest()

            telemetry.addData("NN mode", NN_DETECTOR_MODE.name)
            telemetry.addData(
                "NN detections",
                "%d %s".format(
                    nn.size,
                    if (NeuralReferee.fresh()) "(fresh)" else "(stale)",
                ),
            )
            for (d in nn) {
                telemetry.addData(
                    "  NN %s".format(d.label),
                    String.format(Locale.US, "%.0f%% @ (%.0f,%.0f)", d.confidence * 100, d.x, d.y),
                )
            }
            telemetry.addData("HSV targets (post-referee)", targets.size)
            for (t in targets.take(4)) {
                telemetry.addData(
                    "  HSV %s".format(t.label),
                    String.format(Locale.US, "%.0fcm %+.1fdeg", t.distanceCm, t.bearingDeg),
                )
            }

            if (ShotPredictor.available) {
                telemetry.addData("Battery", "%.2fV".format(ShotPredictor.batteryVolts()))
                for (distCm in listOf(90.0, 110.0, 130.0, 150.0)) {
                    val rpm = ShooterModel.rpmForDistance(distCm)
                    val p = ShotPredictor.pHit(distCm, rpm, ShotPredictor.batteryVolts())
                    telemetry.addData(
                        "P(hit) %dcm".format(distCm.toInt()),
                        "rpm %.0f -> %s".format(rpm, p?.let { "%.0f%%".format(it * 100) } ?: "?"),
                    )
                }
            }
            telemetry.update()
        }

        vision.close()
    }
}