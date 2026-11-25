package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.Time
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/** Pure-JVM regression tests for TeleOp units and no-IMU angular velocity. */
class MecanumDriveTest {

    private val k = MecanumKinematics(1.0, 1.0, 1.0)

    @Test
    fun `teleop full forward maps to +x inches per sec`() {
        val v = MecanumDrive.teleopCommandToVelocity(1.0, 0.0, 0.0)
        assertEquals(DrivetrainSpec.driveFreeSpeedInPerSec, v.component1().x, 1e-9)
        assertEquals(0.0, v.component1().y, 1e-9)
        assertEquals(0.0, v.component2(), 1e-9)
    }

    @Test
    fun `teleop plus strafe right maps to -y because RR +y is left`() {
        val v = MecanumDrive.teleopCommandToVelocity(0.0, 1.0, 0.0)
        assertEquals(0.0, v.component1().x, 1e-9)
        assertEquals(-DrivetrainSpec.driveFreeSpeedInPerSec, v.component1().y, 1e-9)
        assertEquals(0.0, v.component2(), 1e-9)
    }

    @Test
    fun `teleop plus turn clockwise maps to negative angular because RR + is CCW`() {
        val v = MecanumDrive.teleopCommandToVelocity(0.0, 0.0, 1.0)
        assertEquals(0.0, v.component1().x, 1e-9)
        assertEquals(0.0, v.component1().y, 1e-9)
        val expected = -DrivetrainSpec.driveFreeSpeedInPerSec / (DrivetrainSpec.turnRadiusCm() / 2.54)
        assertEquals(expected, v.component2(), 1e-9)
    }

    @Test
    fun `teleop command scales linearly with stick`() {
        val full = MecanumDrive.teleopCommandToVelocity(1.0, 0.0, 0.0)
        val half = MecanumDrive.teleopCommandToVelocity(0.5, 0.0, 0.0)
        assertEquals(full.component1().x / 2.0, half.component1().x, 1e-9)
    }

    @Test
    fun `zero command is zero velocity`() {
        val v = MecanumDrive.teleopCommandToVelocity(0.0, 0.0, 0.0)
        assertEquals(0.0, v.component1().x, 1e-9)
        assertEquals(0.0, v.component1().y, 1e-9)
        assertEquals(0.0, v.component2(), 1e-9)
    }

    @Test
    fun `mecanum omega comes from the DualNum derivative measured wheel speed`() {
        val noDeriv = MecanumKinematics.WheelIncrements(
            DualNum<Time>(doubleArrayOf(1.0, 0.0)),
            DualNum<Time>(doubleArrayOf(1.0, 0.0)),
            DualNum<Time>(doubleArrayOf(1.0, 0.0)),
            DualNum<Time>(doubleArrayOf(1.0, 0.0)),
        )
        assertEquals(0.0, k.forward(noDeriv).velocity().value().component2(), 1e-9)

        val turn = MecanumKinematics.WheelIncrements(
            DualNum<Time>(doubleArrayOf(0.0, 1.0)),
            DualNum<Time>(doubleArrayOf(0.0, 1.0)),
            DualNum<Time>(doubleArrayOf(0.0, -1.0)),
            DualNum<Time>(doubleArrayOf(0.0, -1.0)),
        )
        val omega = k.forward(turn).velocity().value().component2()
        assertTrue("in-place rotation should report nonzero omega, got $omega", omega != 0.0)
    }

    @Test
    fun `all wheels at equal speed reports zero omega`() {
        val straight = MecanumKinematics.WheelIncrements(
            DualNum<Time>(doubleArrayOf(0.0, 2.0)),
            DualNum<Time>(doubleArrayOf(0.0, 2.0)),
            DualNum<Time>(doubleArrayOf(0.0, 2.0)),
            DualNum<Time>(doubleArrayOf(0.0, 2.0)),
        )
        assertEquals(0.0, k.forward(straight).velocity().value().component2(), 1e-9)
    }
}
