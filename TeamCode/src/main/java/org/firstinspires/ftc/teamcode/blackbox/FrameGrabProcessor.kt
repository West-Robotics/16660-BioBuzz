package org.firstinspires.ftc.teamcode.blackbox

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.BLACKBOX_FRAME_CAPTURE_FPS
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import java.io.File

/**
 * Attaches to a VisionPortal alongside the normal vision processors and saves
 * subsampled JPEG frames from the raw camera stream to the Black Box session.
 * The frames are the EXACT images the vision stack saw — timestamps
 * (captureNanos) match the state.jsonl lines, so a computer can replay what
 * the robot "was thinking" frame by frame.
 *
 * Cost control: FRAME_CAPTURE_FPS per camera (default 5) and the session MB
 * budget in globals.kt. Runs on the vision thread and swallows its own errors
 * so it can never stall image processing.
 */
class FrameGrabProcessor(private val camera: String) : VisionProcessor {

    @Volatile
    var enabled = true

    private var lastSaveNanos = 0L
    private var frameCount = 0

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        val f = frame ?: return null
        if (!enabled || !BlackBox.active) return null
        val intervalNanos = (1e9 / BLACKBOX_FRAME_CAPTURE_FPS).toLong()
        if (captureTimeNanos - lastSaveNanos < intervalNanos) return null
        val dir = BlackBox.framesDirFor(camera) ?: return null
        if (!BlackBox.frameBudgetAvailable()) return null

        lastSaveNanos = captureTimeNanos
        return try {
            val name = "frame_%06d.jpg".format(++frameCount)
            val out = File(dir, name)
            Imgcodecs.imwrite(out.absolutePath, f)
            BlackBox.noteFrame(camera, name, captureTimeNanos, out.length().toInt())
            null
        } catch (_: Exception) {
            null
        }
    }

    override fun onDrawFrame(
        canvas: android.graphics.Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
    }
}