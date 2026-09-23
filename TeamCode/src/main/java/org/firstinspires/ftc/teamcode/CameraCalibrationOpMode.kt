package org.firstinspires.ftc.teamcode

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.vision.CameraId
import org.firstinspires.ftc.teamcode.vision.VisionConfig
import org.firstinspires.ftc.teamcode.vision.VisionTuning
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection
import org.opencv.core.Mat
import org.opencv.core.Point
import java.util.Locale
import kotlin.math.atan
import kotlin.math.sqrt

/**
 * ============ C270 CAMERA CALIBRATION (debug GUI) ============
 *
 * Reads the real focal-length numbers for the two Logitech C270s (HIGH/UP cam
 * + GROUND/FRONT cam) straight off the Driver Station — no Android Studio, no
 * guessing. The current values in VisionConfig.kt are marked "MEASURE"; this
 * OpMode is how you measure them.
 *
 * HOW TO USE (5 minutes, one tape measure, one printed AprilTag):
 *  1. Print ANY AprilTag from the 36h11 family that is NOT a BioBuzz hive tag
 *     (IDs 30-45 are hive tags; e.g. use ID 1). Measure the width of the
 *     BLACK SQUARE in mm, edge to edge.
 *  2. Tape it flat to a wall, dead-center in front of one camera, facing the
 *     camera squarely (no tilt).
 *  3. Tape-measure the lens-to-tag distance.
 *  4. Run this OpMode. Both cameras stream stacked; the tag in view gets
 *     outlined, and telemetry shows a LIVE focal estimate.
 *  5. Match your measurements:
 *        DPAD UP/DOWN    = actual distance  (+1 cm; hold X for 10 cm)
 *        DPAD LEFT/RIGHT = printed tag width (+1 mm; hold X for 10 mm)
 *  6. Press A to HOLD — the estimate averages over frames for a stable
 *     number. Press A again to release and re-aim.
 *  7. Write the HELD AVERAGE into VisionConfig.kt:
 *        UP (high) cam  -> UP_CAMERA.intrinsics.focalLengthPx
 *        FRONT (ground) -> FRONT_CAMERA.intrinsics.focalLengthPx
 *     (and C270_FOCAL_LENGTH_PIXELS in globals.kt if you calibrate at 640x480)
 *  8. Sanity checks built into telemetry:
 *     - "dist with configured focal" should read ~ your tape-measure distance
 *     - horizontal FOV should land around 55-60 deg for a C270
 */
@TeleOp(name = "Camera Calibration (C270)", group = "Test")
class CameraCalibrationOpMode : LinearOpMode() {

    private var knownDistanceCm = 100.0
    private var tagWidthMm = APRILTAG_PHYSICAL_WIDTH_MM
    private var holding = false
    private val held = mutableListOf<Double>()
    private var prevA = false

    /** One single-tag measurement (all pixel values at the camera's stream resolution). */
    private data class TagMeasure(
        val id: Int,
        val centerPx: Point,
        /** Axis-aligned corner-quad width, px. */
        val widthPx: Double,
        /** Axis-aligned corner-quad height, px. */
        val heightPx: Double,
        /** Focal length implied by the known distance + printed tag size: f = wPx * dist / size. */
        val focalPx: Double,
    )

    override fun runOpMode() {
        val upCam = hardwareMap.get(WebcamName::class.java, HIGH_CAM_NAME)
        val frontCam = hardwareMap.get(WebcamName::class.java, GROUND_CAM_NAME)

        val upTag = tagProcessor()
        val frontTag = tagProcessor()
        val upOverlay = CalibrationOverlay()
        val frontOverlay = CalibrationOverlay()

        // Stacked live previews for both cameras on the Driver Station.
        val viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL)
        val upPortal = portal(upCam, VisionTuning.UP_CAM_WIDTH, VisionTuning.UP_CAM_HEIGHT, viewIds[0], upTag, upOverlay)
        val frontPortal = portal(frontCam, VisionTuning.FRONT_CAM_WIDTH, VisionTuning.FRONT_CAM_HEIGHT, viewIds[1], frontTag, frontOverlay)

        telemetry.addLine("C270 calibration: show ONE printed (non-hive) AprilTag")
        telemetry.addLine("squarely at one camera; enter distance & tag size.")
        telemetry.addData("Default distance", "%.0f cm", knownDistanceCm)
        telemetry.addData("Default tag width", "%.1f mm (3.25in BioBuzz tag)", tagWidthMm)
        telemetry.addLine("DPAD UP/DOWN: distance | DPAD L/R: tag width | X: 10x")
        telemetry.addLine("A: hold/release averaging")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            handleControls()

            val up = measure(upTag)
            val front = measure(frontTag)
            val upCfg = VisionConfig.calibrationFor(CameraId.UP).intrinsics
            val frontCfg = VisionConfig.calibrationFor(CameraId.FRONT).intrinsics

            upOverlay.text = overlayText("UP (high cam)", up, upCfg.focalLengthPx)
            frontOverlay.text = overlayText("FRONT (ground cam)", front, frontCfg.focalLengthPx)

            if (holding) {
                up?.let { held.add(it.focalPx) }
                front?.let { held.add(it.focalPx) }
            }

            telemetry.addData("Known distance", "%.0f cm", knownDistanceCm)
            telemetry.addData("Printed tag width", "%.1f mm", tagWidthMm)
            telemetry.addLine("----------------------------------------")
            showCamera("UP / high cam", up, VisionTuning.UP_CAM_WIDTH, upCfg.focalLengthPx, "VisionConfig.kt: UP_CAMERA.focalLengthPx")
            telemetry.addLine("----------------------------------------")
            showCamera("FRONT / ground cam", front, VisionTuning.FRONT_CAM_WIDTH, frontCfg.focalLengthPx, "VisionConfig.kt: FRONT_CAMERA.focalLengthPx")
            telemetry.addLine("----------------------------------------")
            showHeld()
            telemetry.update()
        }

        upPortal.close()
        frontPortal.close()
    }

    // ---------------- helpers ----------------

    private fun tagProcessor() = AprilTagProcessor.Builder()
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(false)
        .build()

    private fun portal(
        cam: WebcamName,
        width: Int,
        height: Int,
        viewId: Int,
        processor: AprilTagProcessor,
        overlay: CalibrationOverlay,
    ): VisionPortal = VisionPortal.Builder()
        .setCamera(cam)
        .setCameraResolution(Size(width, height))
        .setLiveViewContainerId(viewId)
        .addProcessors(processor, overlay)
        .build()

    private fun handleControls() {
        val step = if (gamepad1.x) 10.0 else 1.0
        if (gamepad1.dpad_up) {
            knownDistanceCm = (knownDistanceCm + step).coerceIn(10.0, 999.0)
            sleep(120)
        } else if (gamepad1.dpad_down) {
            knownDistanceCm = (knownDistanceCm - step).coerceIn(10.0, 999.0)
            sleep(120)
        }
        if (gamepad1.dpad_right) {
            tagWidthMm = (tagWidthMm + step).coerceIn(20.0, 500.0)
            sleep(120)
        } else if (gamepad1.dpad_left) {
            tagWidthMm = (tagWidthMm - step).coerceIn(20.0, 500.0)
            sleep(120)
        }
        val a = gamepad1.a
        if (a && !prevA) {
            holding = !holding
            if (holding) held.clear()
        }
        prevA = a
    }

    /** Largest single tag in view, with the implied focal length. Clusters are ignored. */
    private fun measure(processor: AprilTagProcessor): TagMeasure? {
        val detections = try {
            processor.detections.toList()
        } catch (_: Exception) {
            emptyList()
        }
        var best: TagMeasure? = null
        var bestArea = 0.0
        for (d in detections) {
            if (d !is AprilTagSingleDetection) continue
            val corners = d.corners ?: continue
            if (corners.size < 4) continue
            val w = corners.maxOf { it.x } - corners.minOf { it.x }
            val h = corners.maxOf { it.y } - corners.minOf { it.y }
            if (w <= 1.0 || h <= 1.0) continue
            if (w * h <= bestArea) continue
            bestArea = w * h
            best = TagMeasure(
                id = d.id,
                centerPx = d.center ?: Point(0.0, 0.0),
                widthPx = w,
                heightPx = h,
                focalPx = w * (knownDistanceCm * 10.0) / tagWidthMm,
            )
        }
        return best
    }

    private fun showCamera(name: String, m: TagMeasure?, camWidth: Int, configuredFocal: Double, writeTarget: String) {
        if (m == null) {
            telemetry.addData(name, "no single tag in view")
            return
        }
        val distWithCfgCm = configuredFocal * tagWidthMm / m.widthPx / 10.0
        val hFovDeg = 2 * Math.toDegrees(atan(camWidth / 2.0 / m.focalPx))
        telemetry.addData(
            "$name tag ${m.id}",
            String.format(Locale.US, "%.0fx%.0f px, center (%.0f, %.0f)", m.widthPx, m.heightPx, m.centerPx.x, m.centerPx.y),
        )
        telemetry.addData(
            "$name LIVE focal",
            String.format(Locale.US, "%.0f px  (configured %.0f, %+.1f%%)", m.focalPx, configuredFocal, (m.focalPx / configuredFocal - 1.0) * 100.0),
        )
        telemetry.addData(
            "$name check",
            String.format(Locale.US, "dist w/ cfg = %.0f cm (actual %.0f), hFOV %.1f deg", distWithCfgCm, knownDistanceCm, hFovDeg),
        )
        telemetry.addData("$name write to", writeTarget)
    }

    private fun showHeld() {
        if (holding) {
            telemetry.addData("HOLD", "averaging... %d samples", held.size)
        } else if (held.isNotEmpty()) {
            val avg = held.average()
            val std = sqrt(held.sumOf { (it - avg) * (it - avg) } / held.size)
            telemetry.addData("HELD AVERAGE", String.format(Locale.US, "%.0f px  (%d samples, +/- %.1f)", avg, held.size, std))
            telemetry.addLine("^^ write this number into VisionConfig.kt")
        } else {
            telemetry.addData("HOLD", "press A to start averaging")
        }
    }

    private fun overlayText(name: String, m: TagMeasure?, configuredFocal: Double): String {
        if (m == null) return "$name\nno tag in view"
        return String.format(
            Locale.US,
            "%s\nfocal ~%.0f px (cfg %.0f)\ntag w=%.0fpx",
            name, m.focalPx, configuredFocal, m.widthPx,
        )
    }
}

/**
 * Draws the live calibration numbers directly on that camera's preview
 * (top-left corner) so the value is readable ON the Driver Station image,
 * next to the outlined tag.
 */
class CalibrationOverlay : VisionProcessor {

    @Volatile
    var text: String = ""

    private var paint: Paint? = null

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? = text

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
        val c = canvas ?: return
        val s = userContext as? String ?: return
        val p = paint ?: Paint().apply {
            color = Color.YELLOW
            textSize = 28f
            isAntiAlias = true
            style = Paint.Style.FILL
            setShadowLayer(3f, 1f, 1f, Color.BLACK)
        }.also { paint = it }
        var y = 40f
        for (line in s.split('\n')) {
            c.drawText(line, 12f, y, p)
            y += p.textSize + 6f
        }
    }
}