package org.firstinspires.ftc.teamcode

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.*
import org.opencv.imgproc.Imgproc

/**
 * VisionProcessor to draw text metadata on the camera stream.
 * Only active when DEBUG_MODE is true.
 */
class MetadataOverlay : VisionProcessor {
    private val paint = Paint().apply {
        color = Color.WHITE
        textSize = 25f
        isAntiAlias = true
        style = Paint.Style.FILL
        setShadowLayer(3f, 1f, 1f, Color.BLACK)
    }

    private var detections = listOf<VisionSystem.Detection>()

    fun updateDetections(newDetections: List<VisionSystem.Detection>) {
        detections = newDetections
    }

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? = null

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        if (!DEBUG_MODE) return
        
        canvas?.let {
            for (detection in detections) {
                val x = detection.screenX * scaleBmpPxToCanvasPx
                val y = detection.screenY * scaleBmpPxToCanvasPx
                
                val text = "${detection.label}\nDist: ${detection.distanceMm.toInt()}mm\nAngle: ${detection.angleDegrees.toInt()}°"
                val lines = text.split("\n")
                var yOffset = 0f
                for (line in lines) {
                    it.drawText(line, x, y - 10f + yOffset, paint)
                    yOffset += paint.textSize + 8f
                }
            }
        }
    }
}

/**
 * CalibrationProcessor allows real-time HSV tweaking.
 * Only active when DEBUG_MODE is true.
 */
class CalibrationProcessor : VisionProcessor {
    var min = Scalar(0.0, 100.0, 100.0)
    var max = Scalar(180.0, 255.0, 255.0)
    
    private val hsv = Mat()
    private val mask = Mat()
    private val hierarchy = Mat()
    private val contours = mutableListOf<MatOfPoint>()

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        if (!DEBUG_MODE) return null
        
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV)
        Core.inRange(hsv, min, max, mask)
        
        contours.clear()
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        
        return contours.map { Imgproc.boundingRect(it) }
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        if (!DEBUG_MODE) return
        
        val rects = userContext as? List<Rect> ?: return
        val paint = Paint().apply {
            color = Color.CYAN
            style = Paint.Style.STROKE
            strokeWidth = 3f
        }
        
        canvas?.let {
            for (rect in rects) {
                val left = rect.x * scaleBmpPxToCanvasPx
                val top = rect.y * scaleBmpPxToCanvasPx
                val right = (rect.x + rect.width) * scaleBmpPxToCanvasPx
                val bottom = (rect.y + rect.height) * scaleBmpPxToCanvasPx
                it.drawRect(left, top, right, bottom, paint)
            }
        }
    }
}
