package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for the autonomous time budget manager (injectable clock).
 */
class PhaseClockTest {

    private var simNanos = 0L
    private val sim: () -> Long get() = { simNanos }

    private fun advance(sec: Double) {
        simNanos += (sec * 1e9).toLong()
    }

    @Test
    fun `counts elapsed and remaining against the period`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        advance(10.0)
        assertEquals(10.0, clock.elapsedSec(), 1e-9)
        assertEquals(20.0, clock.remainingSec(), 1e-9)
    }

    @Test
    fun `remaining never goes negative`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        advance(45.0)
        assertEquals(0.0, clock.remainingSec(), 1e-9)
        assertFalse(clock.fits(1.0))
    }

    @Test
    fun `fits reflects the remaining budget`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        assertTrue(clock.fits(30.0))
        advance(16.0)
        assertFalse(clock.fits(15.0))
        assertTrue(clock.fits(14.0))
    }

    @Test
    fun `phase budget is clamped by what remains after the reserve`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        advance(25.0) // 5s left
        // phase WANTS 9s, but only 5 - 2 (reserve) = 3s are available
        val before = simNanos
        val deadline = clock.deadlineNanos(9000.0, reserveSec = 2.0)
        assertEquals(before + 3.0e9, deadline.toDouble(), 1e6)
    }

    @Test
    fun `phase budget can exceed the phase's own cap`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        advance(2.0) // plenty left
        val before = simNanos
        val deadline = clock.deadlineNanos(4000.0, reserveSec = 2.0)
        // the phase cap (4s) is smaller than the remaining budget -> phase wins
        assertEquals(before + 4.0e9, deadline.toDouble(), 1e6)
    }

    @Test
    fun `no budget at all when the reserve eats everything`() {
        val clock = PhaseClock(periodSec = 30.0, nowNanos = sim)
        advance(28.0) // 2s left, reserve 3s
        val before = simNanos
        val deadline = clock.deadlineNanos(9000.0, reserveSec = 3.0)
        assertEquals(before, deadline) // start nothing
    }
}
