package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for the WorldModel field memory: association, EMA smoothing,
 * expiry, hive memory, and the capacity-aware chase picker.
 */
class WorldModelTest {

    private var nowSec = 0.0
    private val nowNanos: Long get() = (nowSec * 1e9).toLong()

    private fun ball(distCm: Double, bearingDeg: Double, area: Double = 400.0, label: String = "Pollen") =
        Sighting(label, distCm, bearingDeg, area, isBall = true)

    private fun feed(model: WorldModel, vararg sightings: Sighting, x: Double = 0.0, y: Double = 0.0, h: Double = 0.0) {
        model.update(sightings.toList(), x, y, h, nowNanos)
    }

    @Test
    fun `two sightings of the same ball associate into one track`() {
        val m = WorldModel()
        feed(m, ball(100.0, 0.0))
        nowSec += 0.1
        feed(m, ball(102.0, 1.0)) // ~2cm apart — well within WM_TRACK_MATCH_CM
        assertEquals(1, m.balls().size)
    }

    @Test
    fun `two sightings of different balls stay separate`() {
        val m = WorldModel()
        feed(m, ball(100.0, 0.0), ball(100.0, 40.0)) // ~65cm apart — beyond association radius
        assertEquals(2, m.balls().size)
    }

    @Test
    fun `re-sighting moves the track by the EMA fraction, not a jump`() {
        val m = WorldModel()
        feed(m, ball(100.0, 0.0)) // field (100, 0)
        nowSec += 0.5
        feed(m, ball(110.0, 0.0)) // field (110, 0)
        val track = m.balls().first()
        // first sighting sets position directly; the second EMA-steps toward it
        assertEquals(100.0 + WM_TRACK_EMA_ALPHA * 10.0, track.fieldXcm, 1e-6)
    }

    @Test
    fun `removeBall forgets a confirmed pickup`() {
        val m = WorldModel()
        feed(m, ball(80.0, 0.0))
        val id = m.balls().first().id
        m.removeBall(id)
        assertTrue(m.balls().isEmpty())
        assertNull(m.trackById(id))
    }

    @Test
    fun `stale ball tracks expire`() {
        val m = WorldModel()
        feed(m, ball(80.0, 0.0))
        nowSec += WM_BALL_MAX_AGE_SEC + 1.0
        feed(m) // no sightings, just time passing
        assertTrue(m.balls().isEmpty())
    }

    @Test
    fun `robot sightings become opponents with velocity`() {
        val m = WorldModel()
        feed(m, Sighting("Robot", 150.0, 0.0, 5000.0, isBall = false))
        nowSec += 0.5
        feed(m, Sighting("Robot", 145.0, 0.0, 5000.0, isBall = false))
        val opp = m.opponents().first()
        // moved 5cm closer in 0.5s (=-10 cm/s raw); the velocity EMA weights
        // the new sample at 0.3 -> -3 cm/s after one update
        assertEquals(-3.0, opp.velXcmPerSec, 0.1)
    }

    @Test
    fun `hive memory survives tag loss`() {
        val m = WorldModel()
        assertNull(m.hive())
        // a live hive target 100cm dead ahead of the robot at the origin
        m.updateHive(HiveTarget(-1, "RED SCORING", 100.0, 0.0, 0.0, 0.0, 0.0, 0, true, 0), 0.0, 0.0, 0.0, nowNanos)
        val hive = m.hive()
        assertNotNull(hive)
        assertEquals(100.0, hive!!.first, 1e-6)
        assertTrue(m.hiveConfidence() > 0.2)
    }

    @Test
    fun `bestBallToChase gates on freshness and blob size`() {
        val m = WorldModel()
        // a solid fresh ball and a tiny far blob
        feed(m, ball(80.0, 0.0, area = 500.0), ball(200.0, 20.0, area = 100.0))
        assertEquals(80.0, m.bestBallToChase(0.0, 0.0, 3, nowNanos)!!.distanceTo(0.0, 0.0), 1e-6)

        // after the chase-freshness window, only the fresh-seen ball qualifies;
        // make BOTH stale and confirm nothing is chaseable
        nowSec += DEAD_RECKON_MAX_AGE_SEC + 0.5
        assertNull(m.bestBallToChase(0.0, 0.0, 3, nowNanos))
    }

    @Test
    fun `bestBallToChase last-slot rule prefers highest value`() {
        val m = WorldModel()
        // close pollen beats a FAR nectar on utility, but with ONE slot left
        // the higher-value nectar is taken outright
        feed(m, ball(50.0, 0.0, area = 500.0, label = "Pollen"))
        nowSec += 0.05
        feed(m, ball(250.0, 30.0, area = 500.0, label = "Nectar Blue"))
        assertEquals("Pollen", m.bestBallToChase(0.0, 0.0, 2, nowNanos)!!.label)
        assertEquals("Nectar Blue", m.bestBallToChase(0.0, 0.0, 1, nowNanos)!!.label)
    }

    @Test
    fun `relativeToRobot matches live-sighting bearing convention`() {
        val m = WorldModel()
        feed(m, ball(100.0, 0.0)) // track lands at field (100, 0)
        val track = m.balls().first()
        // robot at origin facing +x: ball dead ahead -> bearing 0
        assertEquals(100.0, track.relativeToRobot(0.0, 0.0, 0.0).first, 1e-6)
        assertEquals(0.0, track.relativeToRobot(0.0, 0.0, 0.0).second, 1e-6)
        // robot at origin rotated to face +y (heading +PI/2): the ball at
        // field (100, 0) now sits 90deg to the RIGHT (+ = right)
        assertEquals(90.0, track.relativeToRobot(0.0, 0.0, Math.PI / 2).second, 1e-6)
    }
}
