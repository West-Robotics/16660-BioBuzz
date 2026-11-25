package org.firstinspires.ftc.teamcode

import kotlin.random.Random
import org.firstinspires.ftc.teamcode.MCTSPlanner.SimState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * JVM tests for the MCTS planner.
 *
 * The two properties that matter:
 *
 *  * the SEARCH NEVER MUTATES THE LIVE WORLD. A playout is a hypothetical
 *    future — if it were allowed to remove real balls from the WorldModel or
 *    black out real flower patches, 128 playouts per planner tick would
 *    delete the robot's actual map. The planner therefore keeps its
 *    collection bookkeeping in a private overlay.
 *  * PROGRESSIVE WIDENING keeps the branching factor bounded, so a node with
 *    twenty candidate actions does not explode the tree and starve the
 *    search of depth.
 */
class MCTSPlannerTest {

    private fun pollen(distCm: Double, bearingDeg: Double = 0.0) =
        Sighting("Pollen", distCm, bearingDeg, 500.0, isBall = true)

    private fun nectar(distCm: Double, bearingDeg: Double = 0.0) =
        Sighting("Nectar Blue", distCm, bearingDeg, 500.0, isBall = false)

    private fun world(vararg sightings: Sighting): WorldModel {
        val m = WorldModel()
        m.update(sightings.toList(), 0.0, 0.0, 0.0, 0L)
        return m
    }

    @Test
    fun `the search does not remove balls from the live world`() {
        val m = world(pollen(80.0), pollen(120.0, 20.0), pollen(200.0, -30.0))
        val before = m.balls().size
        assertEquals(3, before)

        val planner = MCTSPlanner(
            playouts = 64, maxDepth = 4, timeBudgetMs = 5000.0, random = Random(1),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)

        assertEquals("the search deleted real balls", before, m.balls().size)
    }

    @Test
    fun `the search does not empty real flower clusters`() {
        val m = world(nectar(80.0), nectar(90.0))
        m.forceClusterRebuild(0L)
        val clustersBefore = m.flowerClusters().size
        val availableBefore = m.availableFlowerClusters(0L).size

        val planner = MCTSPlanner(
            playouts = 64, maxDepth = 4, timeBudgetMs = 5000.0, random = Random(2),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)

        assertEquals("the search changed the real clusters", clustersBefore, m.flowerClusters().size)
        assertEquals(
            "the search depleted a real flower bed",
            availableBefore,
            m.availableFlowerClusters(0L).size,
        )
    }

    @Test
    fun `the requested number of playouts is run`() {
        val m = world(pollen(80.0))
        val planner = MCTSPlanner(
            playouts = 48, maxDepth = 3, timeBudgetMs = 10000.0, random = Random(3),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)
        assertEquals(48, planner.lastPlayoutsDone)
        assertTrue("the search built no nodes", planner.lastNodeCount > 1)
    }

    @Test
    fun `the search respects its time budget`() {
        val m = world(pollen(80.0), pollen(120.0, 20.0), pollen(160.0, -20.0))
        val planner = MCTSPlanner(
            playouts = 100000, maxDepth = 8, timeBudgetMs = 20.0, random = Random(4),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)
        assertTrue(
            "the search ran for ${planner.lastSearchMillis}ms with a 20ms budget",
            planner.lastSearchMillis < 2000.0,
        )
        assertTrue("the budget should have cut the playouts short", planner.lastPlayoutsDone < 100000)
    }

    @Test
    fun `progressive widening keeps the tree far smaller than the action count`() {
        // eight balls on the field: without progressive widening every root
        // node would fan out into eight children on the first playout
        val sightings = ArrayList<Sighting>()
        for (i in 0 until 8) {
            sightings.add(pollen(60.0 + i * 10.0, i * 8.0 - 30.0))
        }
        val m = world(*sightings.toTypedArray())

        val planner = MCTSPlanner(
            playouts = 32, maxDepth = 4, timeBudgetMs = 10000.0, random = Random(5),
        )
        planner.decide(m, 0, 25.0, 0.0, 0.0)

        // 32 playouts with widening: the tree stays a few dozen nodes, not
        // the 8 * 32 = 256 a full fan-out would create
        assertTrue(
            "the tree grew to ${planner.lastNodeCount} nodes",
            planner.lastNodeCount < 32 * 8,
        )
    }

    @Test
    fun `a wider coefficient builds a bigger tree`() {
        val sightings = ArrayList<Sighting>()
        for (i in 0 until 8) {
            sightings.add(pollen(60.0 + i * 10.0, i * 8.0 - 30.0))
        }
        val m = world(*sightings.toTypedArray())

        val narrow = MCTSPlanner(
            playouts = 64, maxDepth = 4,
            progressiveWideningK = 0.2, timeBudgetMs = 10000.0, random = Random(6),
        )
        narrow.decide(m, 0, 25.0, 0.0, 0.0)

        val wide = MCTSPlanner(
            playouts = 64, maxDepth = 4,
            progressiveWideningK = 4.0, timeBudgetMs = 10000.0, random = Random(6),
        )
        wide.decide(m, 0, 25.0, 0.0, 0.0)

        assertTrue(
            "wide (${wide.lastNodeCount}) should not be smaller than narrow (${narrow.lastNodeCount})",
            wide.lastNodeCount >= narrow.lastNodeCount,
        )
    }

    @Test
    fun `progressive widening follows the one-plus-k-sqrt-visits formula`() {
        val planner = MCTSPlanner(progressiveWideningK = 2.0)
        assertEquals(1, planner.maxChildrenForVisits(0))
        assertEquals(3, planner.maxChildrenForVisits(1)) // 1 + floor(2*1)
        assertEquals(7, planner.maxChildrenForVisits(9)) // 1 + floor(2*3)
        assertEquals(21, planner.maxChildrenForVisits(100)) // 1 + floor(2*10)
    }

    @Test
    fun `progressive widening never lets a node go below one child`() {
        val planner = MCTSPlanner(progressiveWideningK = 0.0)
        assertEquals(1, planner.maxChildrenForVisits(0))
        assertEquals(1, planner.maxChildrenForVisits(1000))
    }

    @Test
    fun `the widening limit grows with the visit count`() {
        val planner = MCTSPlanner(progressiveWideningK = MCTS_PROGRESSIVE_WIDENING_K)
        var previous = 0
        for (visits in listOf(0, 1, 4, 16, 64, 256)) {
            val limit = planner.maxChildrenForVisits(visits)
            assertTrue("the limit must not shrink ($visits visits)", limit >= previous)
            previous = limit
        }
    }

    @Test
    fun `the planner returns a usable decision`() {
        val m = world(pollen(80.0))
        val planner = MCTSPlanner(
            playouts = 32, maxDepth = 3, timeBudgetMs = 10000.0, random = Random(7),
        )
        val decision = planner.decide(m, 0, 25.0, 0.0, 0.0)
        assertNotNull(decision)
        assertTrue(decision.rationale.isNotEmpty())
    }

    @Test
    fun `with nothing on the field the planner does not ask to chase`() {
        val m = WorldModel()
        val planner = MCTSPlanner(
            playouts = 16, maxDepth = 3, timeBudgetMs = 10000.0, random = Random(8),
        )
        val decision = planner.decide(m, 0, 25.0, 0.0, 0.0)
        assertNotNull(decision)
        assertTrue(
            "unexpected action ${decision.action}",
            decision.action != PlannerAction.COLLECT_BALL,
        )
    }

    @Test
    fun `the default planner runs the tuned playout count`() {
        val planner = MCTSPlanner()
        assertEquals(128, planner.playouts)
        assertEquals(4, planner.maxDepth)
        assertEquals(MCTS_PROGRESSIVE_WIDENING_K, planner.progressiveWideningK, 1e-9)
        assertEquals(MCTS_HEURISTIC_WEIGHT, planner.heuristicWeight, 1e-9)
    }

    /** A [Random] whose [nextDouble] draws come from a fixed script, in order. */
    private class ScriptedRandom(private val script: List<Double>) : Random() {
        private var next = 0
        override fun nextDouble(): Double = script.getOrElse(next++) { 0.0 }
        override fun nextInt(until: Int): Int = 0
        override fun nextBits(count: Int): Int = 0
    }

    // --- MCTS outcome-consistency behavior tests (would fail before the fix) ---

    @Test
    fun `failed pickup does not increment inventory or mark the track collected`() {
        val m = world(pollen(80.0))
        val ball = m.balls()[0]
        // 1.0 < any success probability -> pickup always fails
        val planner = MCTSPlanner(random = ScriptedRandom(listOf(1.0)), timeBudgetMs = 10000.0)
        val state = SimState(m, 0, 25.0, 0.0, 0.0)
        val action = PlannerDecision(
            PlannerAction.COLLECT_BALL, ball.id, ball.label, ball.fieldXcm, ball.fieldYcm,
            ball.value * ball.confidence, 1.5, "collect",
        )
        val outcome = planner.resolveAction(action, state)
        assertFalse("pickup should have failed", outcome.succeeded)
        assertEquals(0.0, outcome.immediateValue, 1e-9)

        val next = planner.applyAction(action, state, outcome)
        assertEquals("failed pickup must not grow inventory", 0, next.ballsHeld)
        assertFalse("failed pickup must not mark track collected", ball.id in next.collectedBallIds)
    }

    @Test
    fun `failed score does not empty the hopper`() {
        val m = WorldModel()
        // 1.0 < any scoring probability -> score always fails
        val planner = MCTSPlanner(random = ScriptedRandom(listOf(1.0)), timeBudgetMs = 10000.0)
        val state = SimState(m, 1, 25.0, 0.0, 0.0) // holding one ball
        val action = PlannerDecision(
            PlannerAction.GO_SCORE, -1, "Hive", 0.0, 0.0, POLLEN_VALUE, 7.0, "score",
        )
        val outcome = planner.resolveAction(action, state)
        assertFalse("score should have failed", outcome.succeeded)

        val next = planner.applyAction(action, state, outcome)
        assertEquals("failed score must not empty hopper", 1, next.ballsHeld)
    }

    @Test
    fun `a full hopper cannot choose collect`() {
        val now = System.nanoTime()
        val m = WorldModel()
        m.update(listOf(pollen(80.0), pollen(120.0)), 0.0, 0.0, 0.0, now)
        val planner = MCTSPlanner(random = Random(1), timeBudgetMs = 10000.0)
        val state = SimState(m, MAX_BALLS_HELD, 25.0, 0.0, 0.0) // hopper full
        val actions = planner.generateActions(state)
        assertFalse(
            "COLLECT_BALL must not be offered with a full hopper",
            actions.any { it.action == PlannerAction.COLLECT_BALL },
        )
    }

    @Test
    fun `collect then score rewards each ball exactly once, not twice`() {
        val m = world(pollen(80.0))
        val ball = m.balls()[0]
        // No opponents are present, so collect draws one random (0.0 < successProb -> succeeds)
        // and score draws one random (0.5 < scoreProb -> succeeds).
        val planner = MCTSPlanner(random = ScriptedRandom(listOf(0.0, 0.5)), timeBudgetMs = 10000.0)
        val state = SimState(m, 0, 25.0, 0.0, 0.0)

        val collect = PlannerDecision(
            PlannerAction.COLLECT_BALL, ball.id, ball.label, ball.fieldXcm, ball.fieldYcm,
            ball.value * ball.confidence, 1.5, "collect",
        )
        val collectOutcome = planner.resolveAction(collect, state)
        assertTrue("collect should succeed", collectOutcome.succeeded)
        // Collection awards no immediate points (previously it awarded full value).
        assertEquals(0.0, collectOutcome.immediateValue, 1e-9)

        val afterCollect = planner.applyAction(collect, state, collectOutcome)
        assertEquals(1, afterCollect.ballsHeld)

        val score = PlannerDecision(
            PlannerAction.GO_SCORE, -1, "Hive", 0.0, 0.0, POLLEN_VALUE, 7.0, "score",
        )
        val scoreOutcome = planner.resolveAction(score, afterCollect)
        assertTrue("score should succeed", scoreOutcome.succeeded)
        assertEquals(POLLEN_VALUE, scoreOutcome.immediateValue, 1e-9)

        // Value awarded across collect+score = POLLEN_VALUE exactly once, never
        // POLLEN_VALUE at collection plus POLLEN_VALUE again at scoring.
        assertEquals(POLLEN_VALUE, collectOutcome.immediateValue + scoreOutcome.immediateValue, 1e-9)
    }

    @Test
    fun `flower nectar is credited once at its own value`() {
        val m = WorldModel()
        val planner = MCTSPlanner(random = ScriptedRandom(listOf(0.0)), timeBudgetMs = 10000.0)
        val state = SimState(m, 0, 25.0, 0.0, 0.0)
        val flower = PlannerDecision(
            PlannerAction.USE_FLOWER, 42, "Flower", 20.0, 0.0,
            NECTAR_VALUE, 2.0, "flower",
        )
        val flowerOutcome = planner.resolveAction(flower, state)
        assertEquals(0.0, flowerOutcome.immediateValue, 1e-9)
        val afterFlower = planner.applyAction(flower, state, flowerOutcome)
        assertEquals(NECTAR_VALUE, afterFlower.heldScoreValue, 1e-9)

        val score = PlannerDecision(
            PlannerAction.GO_SCORE, -1, "Hive", 0.0, 0.0, NECTAR_VALUE, 7.0, "score",
        )
        val scoreOutcome = planner.resolveAction(score, afterFlower)
        assertEquals(NECTAR_VALUE, scoreOutcome.immediateValue, 1e-9)
        assertEquals(NECTAR_VALUE, flowerOutcome.immediateValue + scoreOutcome.immediateValue, 1e-9)
    }

    @Test
    fun `expanded successful score retains its reward and beats holding`() {
        val m = WorldModel()
        m.updateHive(
            HiveTarget(-1, "RED SCORING", 100.0, 0.0, 0.0, 0.0, 0.0, 0, true, 0),
            0.0, 0.0, 0.0, System.nanoTime(),
        )
        val planner = MCTSPlanner(
            playouts = 64, maxDepth = 2, timeBudgetMs = 10000.0,
            random = ScriptedRandom(listOf(0.0)),
        )
        val decision = planner.decide(m, ballsHeld = 1, timeRemainingSec = 25.0, robotXcm = 0.0, robotYcm = 0.0)
        assertEquals(PlannerAction.GO_SCORE, decision.action)
        assertTrue("score reward was lost during expansion", decision.expectedValue > 0.0)
    }
}
