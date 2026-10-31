package org.firstinspires.ftc.teamcode.vision

import android.util.Log
import org.firstinspires.ftc.teamcode.FRAME_CAPTURE_DIR
import org.firstinspires.ftc.teamcode.FRAME_CAPTURE_ENABLED
import org.firstinspires.ftc.teamcode.FRAME_CAPTURE_FPS
import org.firstinspires.ftc.teamcode.FRAME_CAPTURE_MAX_MB
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicReference

class FrameGrabProcessor(
    private val cameraName: String,
    private val maxFps: Double = FRAME_CAPTURE_FPS.toDouble(),
    private val maxTotalBytes: Long = FRAME_CAPTURE_MAX_MB.toLong() * 1024L * 1024L,
) : VisionProcessor {

    private val enabled = FRAME_CAPTURE_ENABLED
    private val intervalNanos = (1e9 / maxFps).toLong()
    private val lastSaveNanos = AtomicLong(0L)
    private val frameCounter = AtomicLong(0L)
    private val totalBytes = AtomicLong(0L)
    private val dirRef = AtomicReference<File?>(null)

    @Volatile
    var captureEnabled = true

    override fun init(width: Int, height: Int, calibration: org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration?) {
        try {
            val baseDir = File(android.os.Environment.getExternalStorageDirectory(), "FIRST/$FRAME_CAPTURE_DIR/$cameraName")
            if (!baseDir.exists()) baseDir.mkdirs()
            dirRef.set(baseDir)
            Log.i("FrameGrab", "$cameraName: saving to ${baseDir.absolutePath}")
        } catch (e: Exception) {
            Log.w("FrameGrab", "$cameraName: cannot create output dir: ${e.message}")
        }
        Log.i("FrameGrab", "$cameraName: initialized (${width}x${height})")
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        if (!enabled || !captureEnabled) return null
        val f = frame ?: return null

        val now = captureTimeNanos
        if (now - lastSaveNanos.get() < intervalNanos) return null

        if (totalBytes.get() >= maxTotalBytes) return null

        val dir = dirRef.get() ?: return null
        if (!dir.exists() || !dir.canWrite()) return null

        val bgr = Mat()
        try {
            org.opencv.imgproc.Imgproc.cvtColor(f, bgr, org.opencv.imgproc.Imgproc.COLOR_RGBA2BGR)

            val seq = frameCounter.incrementAndGet()
            val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss_SSS", Locale.US).format(Date(now / 1_000_000))
            val fileName = "frame_${timestamp}_$seq.jpg"
            val outFile = File(dir.absolutePath, fileName)

            if (Imgcodecs.imwrite(outFile.absolutePath, bgr)) {
                val bytes = outFile.length()
                totalBytes.addAndGet(bytes)
                lastSaveNanos.set(now)
                Log.d("FrameGrab", "$cameraName: saved $fileName (${bytes} bytes, total ${totalBytes.get() / 1024} KB)")
            }
        } catch (e: Exception) {
            Log.w("FrameGrab", "$cameraName: save failed: ${e.message}")
        } finally {
            bgr.release()
        }
        return null
    }

    override fun onDrawFrame(
        canvas: android.graphics.Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {}

    fun setCaptureEnabledState(enabled: Boolean) {
        captureEnabled = enabled
    }

    fun getStats(): String {
        return "$cameraName: frames=${frameCounter.get()}, ${totalBytes.get() / 1024} KB used"
    }
}