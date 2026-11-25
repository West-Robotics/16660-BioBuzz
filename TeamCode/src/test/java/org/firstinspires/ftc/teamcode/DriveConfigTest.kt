package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * Pure-JVM regression tests for [DriveConfig] — the single drive polarity + mixing
 * table now shared by the NextFTC [Drivetrain] (teleop) and Road Runner
 * [roadrunner.MecanumDrive] (autonomous/localization).
 *
 * Both drive paths must agree on the wheel commands, so this test pins the
 * mixing matrix (and the right-side-reversed polarity) that makes commanding
 * +1 to all four wheels drive the robot FORWARD on this mirrored mecanum.
 */
class DriveConfigTest {

    @Test
    fun `full forward commands all wheels the same direction`() {
        val w = DriveConfig.mix(1.0, 0.0, 0.0)
        assertArrayEquals(doubleArrayOf(1.0, 1.0, 1.0, 1.0), w, 1e-9)
    }

    @Test
    fun `full strafe right mirrors left vs right wheels`() {
        val w = DriveConfig.mix(0.0, 1.0, 0.0)
        // FL +, FR -, BL -, BR +
        assertArrayEquals(doubleArrayOf(1.0, -1.0, -1.0, 1.0), w, 1e-9)
    }

    @Test
    fun `full turn right mirrors diagonals`() {
        val w = DriveConfig.mix(0.0, 0.0, 1.0)
        // FL +, FR -, BL +, BR -
        assertArrayEquals(doubleArrayOf(1.0, -1.0, 1.0, -1.0), w, 1e-9)
    }

    @Test
    fun `mixing is normalized so no wheel exceeds full power`() {
        val w = DriveConfig.mix(1.0, 1.0, 1.0)
        // FL = (1+1+1)/3 = 1, FR = (1-1-1)/3 = -1/3, BL = (1-1+1)/3 = 1/3,
        // BR = (1+1-1)/3 = 1/3
        assertArrayEquals(doubleArrayOf(1.0, -1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0), w, 1e-9)
    }

    @Test
    fun `oversized inputs are clamped by the denominator`() {
        val w = DriveConfig.mix(2.0, 0.0, 0.0)
        assertArrayEquals(doubleArrayOf(1.0, 1.0, 1.0, 1.0), w, 1e-9)
    }

    @Test
    fun `zero input yields zero wheel commands`() {
        val w = DriveConfig.mix(0.0, 0.0, 0.0)
        assertArrayEquals(doubleArrayOf(0.0, 0.0, 0.0, 0.0), w, 1e-9)
    }
}
