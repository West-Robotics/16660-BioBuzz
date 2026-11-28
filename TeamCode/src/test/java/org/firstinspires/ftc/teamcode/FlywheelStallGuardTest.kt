package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Test

class FlywheelStallGuardTest {
    private fun guard() = FlywheelStallGuard(
        spinupGraceNanos = 1_000,
        stallSustainNanos = 500,
    )

    @Test
    fun `normal spin-up does not trip during grace`() {
        val guard = guard()

        assertEquals(FlywheelStallGuard.State.SPINNING_UP, guard.update(3000.0, 0.0, 4.0, 0))
        assertEquals(FlywheelStallGuard.State.SPINNING_UP, guard.update(3000.0, 50.0, 4.0, 900))
        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(3000.0, 2800.0, 1.0, 1_000))
    }

    @Test
    fun `sustained loaded stall trips and remains latched`() {
        val guard = guard()
        guard.update(3000.0, 2800.0, 1.0, 0)
        guard.update(3000.0, 2800.0, 1.0, 1_000)

        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(3000.0, 50.0, 4.0, 1_100))
        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(3000.0, 50.0, 4.0, 1_599))
        assertEquals(FlywheelStallGuard.State.STALLED, guard.update(3000.0, 50.0, 4.0, 1_600))
        assertEquals(FlywheelStallGuard.State.STALLED, guard.update(0.0, 0.0, 0.0, 2_000))
    }

    @Test
    fun `brief ball-load dip resets the stall timer`() {
        val guard = guard()
        guard.update(3000.0, 2800.0, 1.0, 0)
        guard.update(3000.0, 2800.0, 1.0, 1_000)
        guard.update(3000.0, 50.0, 4.0, 1_100)
        guard.update(3000.0, 2700.0, 1.0, 1_400)

        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(3000.0, 50.0, 4.0, 1_500))
        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(3000.0, 50.0, 4.0, 1_999))
    }

    @Test
    fun `low rpm without current does not indicate an overheating stall`() {
        val guard = guard()
        guard.update(3000.0, 2800.0, 1.0, 0)

        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(3000.0, 0.0, 0.2, 5_000))
    }

    @Test
    fun `missing current sensor still has rpm-only fallback`() {
        val guard = guard()
        guard.update(3000.0, 2800.0, 1.0, 0)
        guard.update(3000.0, 2800.0, 1.0, 1_000)
        guard.update(3000.0, 0.0, null, 1_100)

        assertEquals(FlywheelStallGuard.State.STALLED, guard.update(3000.0, 0.0, null, 1_600))
    }

    @Test
    fun `new speed target receives a fresh spin-up grace`() {
        val guard = guard()
        guard.update(3000.0, 2800.0, 1.0, 0)
        guard.update(3000.0, 2800.0, 1.0, 1_000)

        assertEquals(FlywheelStallGuard.State.SPINNING_UP, guard.update(5000.0, 1000.0, 4.0, 1_100))
        assertEquals(FlywheelStallGuard.State.SPINNING_UP, guard.update(5000.0, 1000.0, 4.0, 2_000))
        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(5000.0, 1000.0, 4.0, 2_100))
    }

    @Test
    fun `oscillating targets cannot postpone a real stall forever`() {
        val guard = guard()
        guard.update(3000.0, 2800.0, 1.0, 0)
        guard.update(3000.0, 2800.0, 1.0, 1_000)
        guard.update(3000.0, 50.0, 4.0, 1_100)

        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(5000.0, 50.0, 4.0, 1_200))
        assertEquals(FlywheelStallGuard.State.RUNNING, guard.update(3000.0, 50.0, 4.0, 1_300))
        assertEquals(FlywheelStallGuard.State.STALLED, guard.update(5000.0, 50.0, 4.0, 1_600))
    }

    @Test
    fun `explicit reset permits a fresh spin-up`() {
        val guard = guard()
        guard.update(3000.0, 2800.0, 1.0, 0)
        guard.update(3000.0, 2800.0, 1.0, 1_000)
        guard.update(3000.0, 0.0, 4.0, 1_100)
        guard.update(3000.0, 0.0, 4.0, 1_600)

        guard.reset()

        assertEquals(FlywheelStallGuard.State.SPINNING_UP, guard.update(3000.0, 0.0, 4.0, 2_000))
    }
}
