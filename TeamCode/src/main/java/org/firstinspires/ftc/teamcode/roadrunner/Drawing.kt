package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d

/** Dashboard robot drawing helpers (Road Runner 1.0 quickstart utility). */
object Drawing {
    private const val ROBOT_RADIUS_IN = 9.0

    fun drawRobot(c: Canvas, t: Pose2d) {
        c.setStrokeWidth(1)
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS_IN)

        val halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS_IN)
        val p1 = t.position.plus(halfv)
        val p2 = p1.plus(halfv)
        c.strokeLine(p1.x, p1.y, p2.x, p2.y)
    }
}
