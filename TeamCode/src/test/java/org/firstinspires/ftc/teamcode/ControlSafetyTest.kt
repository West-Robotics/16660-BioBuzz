package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ControlSafetyTest {
    @Test
    fun `intervention expires and stays latched until request clears`() {
        val intervention = BoundedIntervention(maxDurationNanos = 1_000)

        assertTrue(intervention.allowed(true, 0))
        assertTrue(intervention.allowed(true, 999))
        assertFalse(intervention.allowed(true, 1_000))
        assertFalse(intervention.allowed(true, 2_000))
        assertFalse(intervention.allowed(false, 2_100))
        assertTrue(intervention.allowed(true, 2_200))
    }

    @Test
    fun `intervention does not restart while request remains active`() {
        val intervention = BoundedIntervention(maxDurationNanos = 100)

        assertTrue(intervention.allowed(true, 10_000))
        assertFalse(intervention.allowed(true, 10_100))
        assertFalse(intervention.allowed(true, 50_000))
    }

    @Test
    fun `encoder motion uses a real delta threshold`() {
        assertFalse(encoderMoved(100, 129, 30))
        assertTrue(encoderMoved(100, 130, 30))
        assertTrue(encoderMoved(100, 70, 30))
        assertFalse(encoderMoved(0, 0, 30))
    }

    @Test
    fun `encoder delta cannot overflow at integer limits`() {
        assertTrue(encoderMoved(Int.MIN_VALUE, Int.MAX_VALUE, 30))
    }
}
