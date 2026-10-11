package org.firstinspires.ftc.teamcode.nn

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.NN_DETECTOR_MODE
import org.firstinspires.ftc.teamcode.NN_DETECT_CONFIDENCE
import org.firstinspires.ftc.teamcode.NN_DETECT_HZ
import org.firstinspires.ftc.teamcode.NN_DETECT_IOU
import org.firstinspires.ftc.teamcode.NN_MODEL_FILE
import org.firstinspires.ftc.teamcode.NnMode
import org.firstinspires.ftc.teamcode.blackbox.BlackBox
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.tensorflow.lite.Interpreter
import java.io.File
import java.io.FileInputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel

/**
 * ============ BALL DETECTOR — tiny YOLO via TFLite, fully on-robot ============
 *
 * Neural net #2 (A of the staged plan). Attaches to the FRONT camera portal
 * alongside the HSV processors and runs a float-I/O TFLite detector at
 * NN_DETECT_HZ (default 3 — the hub CPU also runs the rest of vision, so
 * keep this LOW). Detections are published to [NeuralReferee], which the
 * BallLocator consults per the authority ladder (OFF / ADVISORY / REFEREE /
 * PRIMARY — see globals.kt).
 *
 * Train with tools/auto_label.py (+ tools/label_review.py to fix labels) and
 * tools/train_detector.py; copy ball_detector.tflite to the app's external
 * storage (the same place blackbox/ lives) over USB.
 */
class DetectorProcessor(context: Context) : VisionProcessor {

    private var interpreter: Interpreter? = null
    private var inputSize = 320
    private var inputBuf: ByteBuffer? = null
    private var outputShape: IntArray = IntArray(0)
    private var lastNanos = 0L
    private var paint: Paint? = null

    init {
        if (NN_DETECTOR_MODE != NnMode.OFF) {
            try {
                val f = File(context.getExternalFilesDir(null), NN_MODEL_FILE)
                if (!f.exists()) {
                    BlackBox.event("nn_status", "component" to "detector", "status" to "no model file")
                } else {
                    val channel = FileInputStream(f).channel
                    val model: MappedByteBuffer =
                        channel.map(FileChannel.MapMode.READ_ONLY, 0, f.length())
                    interpreter = Interpreter(model, Interpreter.Options().apply { numThreads = 2 })
                    val inShape = interpreter!!.getInputTensor(0).shape() // [1, S, S, 3]
                    inputSize = inShape[1]
                    inputBuf =
                        ByteBuffer.allocateDirect(inputSize * inputSize * 3 * 4)
                            .order(ByteOrder.nativeOrder())
                    outputShape = interpreter!!.getOutputTensor(0).shape()
                    BlackBox.event(
                        "nn_status", "component" to "detector",
                        "status" to "loaded", "inputSize" to inputSize,
                        "output" to outputShape.joinToString("x"),
                    )
                }
            } catch (e: Exception) {
                BlackBox.event(
                    "nn_status", "component" to "detector",
                    "status" to "error", "msg" to (e.message ?: "?"),
                )
            }
        }
    }

    val loaded: Boolean get() = interpreter != null

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {}

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any? {
        val itp = interpreter ?: return null
        val f = frame ?: return null
        if (captureTimeNanos - lastNanos < (1e9 / NN_DETECT_HZ).toLong()) return null
        lastNanos = captureTimeNanos

        return try {
            // 1. resize to the model's square input (aspect stretched back later)
            val resized = Mat()
            Imgproc.resize(f, resized, Size(inputSize.toDouble(), inputSize.toDouble()))
            // 2. pixels -> float32 NHWC buffer in [0,1], RGB order like training
            val bytes = ByteArray(inputSize * inputSize * 3)
            resized.get(0, 0, bytes)
            val buf = inputBuf!!
            buf.rewind()
            var i = 0
            while (i < bytes.size) {
                buf.putFloat((bytes[i].toInt() and 0xFF) / 255f)       // R
                buf.putFloat((bytes[i + 2].toInt() and 0xFF) / 255f)   // G (BGR->RGB)
                buf.putFloat((bytes[i + 1].toInt() and 0xFF) / 255f)   // B
                i += 3
            }
            // 3. allocate output by the model's real shape and run
            val out = Array(outputShape[0]) {
                Array(outputShape[1]) { FloatArray(outputShape[2]) }
            }
            itp.run(buf, out)
            // 4. decode + NMS + publish (boxes scaled back to full-frame coords)
            NeuralReferee.publish(decode(out, f.cols().toDouble(), f.rows().toDouble()), captureTimeNanos)
            null
        } catch (_: Exception) {
            null // inference must never stall the vision pipeline
        }
    }

    /**
     * Decodes ultralytics-style TFLite output [1][4+nc][N] (cx, cy, w, h in
     * input pixels, then per-class scores) with greedy NMS. Boxes come back
     * in FULL-FRAME pixel coordinates.
     */
    private fun decode(out: Array<Array<FloatArray>>, frameW: Double, frameH: Double): List<NeuralReferee.Detection> {
        val channels = out[0].size
        val n = out[0][0].size
        val numClasses = channels - 4
        if (numClasses <= 0) return emptyList()

        val sx = frameW / inputSize
        val sy = frameH / inputSize
        val raw = ArrayList<DoubleArray>() // [cx, cy, w, h, score, cls]
        for (j in 0 until n) {
            var bestScore = 0.0
            var bestCls = -1
            for (c in 0 until numClasses) {
                val s = out[0][4 + c][j].toDouble()
                if (s > bestScore) {
                    bestScore = s; bestCls = c
                }
            }
            if (bestScore < NN_DETECT_CONFIDENCE) continue
            raw.add(doubleArrayOf(out[0][0][j] * sx, out[0][1][j] * sy,
                out[0][2][j] * sx, out[0][3][j] * sy, bestScore, bestCls.toDouble()))
        }
        raw.sortByDescending { it[4] }

        val kept = ArrayList<DoubleArray>()
        outer@ for (b in raw) {
            for (k in kept) {
                val x1 = maxOf(b[0] - b[2] / 2, k[0] - k[2] / 2)
                val y1 = maxOf(b[1] - b[3] / 2, k[1] - k[3] / 2)
                val x2 = minOf(b[0] + b[2] / 2, k[0] + k[2] / 2)
                val y2 = minOf(b[1] + b[3] / 2, k[1] + k[3] / 2)
                val inter = maxOf(0.0, x2 - x1) * maxOf(0.0, y2 - y1)
                if (inter / (b[2] * b[3] + k[2] * k[3] - inter) > NN_DETECT_IOU) continue@outer
            }
            kept.add(b)
        }

        return kept.take(12).map {
            NeuralReferee.Detection(
                labelId = it[5].toInt(),
                confidence = it[4],
                x = it[0] - it[2] / 2, y = it[1] - it[3] / 2,
                w = it[2], h = it[3],
            )
        }
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?,
    ) {
        val c = canvas ?: return
        val stroke = paint ?: Paint().apply {
            textSize = 24f
            isAntiAlias = true
            style = Paint.Style.STROKE
            strokeWidth = 3f
        }.also { paint = it }
        if (!NeuralReferee.fresh()) return
        val text = Paint().apply {
            textSize = 22f
            color = Color.rgb(0, 200, 255)
            style = Paint.Style.FILL
        }
        for (d in NeuralReferee.latest()) {
            stroke.color = Color.rgb(0, 200, 255)
            c.drawRect(
                (d.x * scaleBmpPxToCanvasPx).toFloat(),
                (d.y * scaleBmpPxToCanvasPx).toFloat(),
                ((d.x + d.w) * scaleBmpPxToCanvasPx).toFloat(),
                ((d.y + d.h) * scaleBmpPxToCanvasPx).toFloat(),
                stroke,
            )
            c.drawText(
                "NN %s %.0f%%".format(
                    org.firstinspires.ftc.teamcode.NN_LABELS.getOrElse(d.labelId) { "?" },
                    d.confidence * 100,
                ),
                (d.x * scaleBmpPxToCanvasPx).toFloat() + 4f,
                (d.y * scaleBmpPxToCanvasPx).toFloat() - 6f,
                text,
            )
        }
    }
}