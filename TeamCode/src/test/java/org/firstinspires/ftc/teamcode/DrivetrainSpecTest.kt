package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.PI

/**
 * Verifies the measurement -> constant derivations that the whole drivetrain
 * (encoder conversions, Road Runner defaults, dead-wheel geometry) consumes.
 */
class DrivetrainSpecTest {

    @Test
    fun `ticks per wheel rev is CPR times gear ratio`() {
        // 28-count motor encoder through the 51.46:1 gearbox
        assertEquals(28.0 * 51.46, DrivetrainSpec.driveTicksPerWheelRev, 1e-9)
    }

    @Test
    fun `one wheel revolution travels pi times the diameter`() {
        val ticksPerRev = DrivetrainSpec.driveTicksPerWheelRev
        val cm = DrivetrainSpec.driveCmPerTick * ticksPerRev
        assertEquals(PI * DrivetrainSpec.DRIVE_WHEEL_DIAMETER_CM, cm, 1e-9)
    }

    @Test
    fun `cm to ticks and back round-trips`() {
        val ticks = DrivetrainSpec.cmToWheelTicks(100.0)
        assertEquals(100.0, ticks * DrivetrainSpec.driveCmPerTick, 0.5) // half-tick tolerance
    }

    @Test
    fun `wheel radians never include the gear ratio`() {
        // NextFTC positions are in WHEEL radians: a full wheel turn = 2π rad.
        assertEquals(2.0 * PI, DrivetrainSpec.cmToWheelRad(PI * DrivetrainSpec.DRIVE_WHEEL_DIAMETER_CM), 1e-9)
    }

    @Test
    fun `road runner defaults are consistent with the geometry`() {
        // inPerTick converts a tick to inches; trackWidthTicks converts the
        // track width to ticks — their product must give the track in inches.
        val trackIn = DrivetrainSpec.TRACK_WIDTH_CM / 2.54
        assertEquals(trackIn, DrivetrainSpec.rrInPerTick * DrivetrainSpec.rrTrackWidthTicks, 1e-9)
        // lateralInPerTick scales by the slip factor only.
        assertEquals(DrivetrainSpec.rrInPerTick * DrivetrainSpec.MECANUM_LATERAL_SLIP, DrivetrainSpec.rrLateralInPerTick, 1e-12)
    }

    @Test
    fun `dead wheel geometry converts cm to inches with the right angles`() {
        val wheels = DrivetrainSpec.deadWheelsIn()
        assertEquals(3, wheels.size)
        assertEquals(DrivetrainSpec.PAR0_X_CM / 2.54, wheels[0].x, 1e-12)
        assertEquals(DrivetrainSpec.PAR0_Y_CM / 2.54, wheels[0].y, 1e-12)
        assertEquals(0.0, wheels[0].angleRad, 1e-12) // par wheels roll with driving
        assertEquals(0.0, wheels[1].angleRad, 1e-12)
        assertEquals(PI / 2, wheels[2].angleRad, 1e-12) // perp rolls with strafing
        assertEquals(DrivetrainSpec.PAR1_Y_CM / 2.54, wheels[1].y, 1e-12)
        assertEquals(DrivetrainSpec.PERP_X_CM / 2.54, wheels[2].x, 1e-12)
    }

    @Test
    fun `dead wheel inches per tick derives from pod diameter`() {
        assertEquals(
            PI * DrivetrainSpec.DEAD_WHEEL_DIAMETER_CM / 2.54 / DrivetrainSpec.DEAD_WHEEL_TICKS_PER_REV,
            DrivetrainSpec.deadWheelInPerTick,
            1e-12,
        )
    }

    @Test
    fun `turn radius follows the wheelbase and track width`() {
        val expected = kotlin.math.sqrt(
            DrivetrainSpec.WHEELBASE_CM * DrivetrainSpec.WHEELBASE_CM +
                DrivetrainSpec.TRACK_WIDTH_CM * DrivetrainSpec.TRACK_WIDTH_CM,
        ) / 2.0
        assertEquals(expected, DrivetrainSpec.turnRadiusCm(), 1e-9)
    }
}
