package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.blackbox.BlackBox
import kotlin.math.hypot

/**
 * ============ STRATEGIC PLANNER — the lookahead brain ============
 *
 * Replaces the reactive "what do I see RIGHT NOW" strategy enum with a
 * value/cost search over the WorldModel's memory:
 *
 *  * every candidate action (collect ball i / go score / use flower / park)
 *    gets an expected value and an estimated time cost,
 *  * balls an opponent is closer to get discounted (they'll take them),
 *  * the ENDGAME is modeled: nothing starts that can't pay back before the
 *    clock runs out — full payloads force a score run, the last seconds
 *    force park.
 *
 * Deliberately small and deterministic (runs in microseconds) — the structure
 * is ready for a deeper search (MCTS) later, but this level already beats the
 * greedy controller because it remembers unseen balls and budgets time.
 *
 * It also powers SHADOW MODE: called during human teleop it logs what it
 * would do (without acting) — that log is how we'll prove it outplays the
 * human before ever letting it drive.
 */
enum class PlannerAction { COLLECT_BALL, GO_SCORE, USE_FLOWER, PARK, HOLD }

data class PlannerDecision(
    val action: PlannerAction,
    val targetId: Int,
    val targetLabel: String,
    val targetFieldXcm: Double,
    val targetFieldYcm: Double,
    val expectedValue: Double,
    val estCostSec: Double,
    val rationale: String,
)

class StrategicPlanner {

    /**
     * Picks the best action given the remembered world and our own state.
     *
     * @param world          persistent field memory (tracks + hive)
     * @param ballsHeld      balls currently in the robot
     * @param timeRemainingSec  seconds until the match ends — never start
     *                       anything that can't finish
     * @param robotXcm/Ycm   current robot position (field cm)
     */
    fun decide(
        world: WorldModel,
        ballsHeld: Int,
        timeRemainingSec: Double,
        robotXcm: Double,
        robotYcm: Double,
    ): PlannerDecision {
        val hive = world.hive()
        val distToHive =
            if (hive != null) hypot(hive.first - robotXcm, hive.second - robotYcm) else 150.0
        val scoreTime = distToHive / ROBOT_SPEED_CM_PER_SEC + PLANNER_SCORE_TOTAL_SEC

        // ---- 1. ENDGAME: the clock is the strongest constraint ----
        if (timeRemainingSec <= PLANNER_PARK_SEC + 2.0) {
            return PlannerDecision(
                PlannerAction.PARK, -1, "", 0.0, 0.0, 0.0, PLANNER_PARK_SEC,
                "PARK: %.0fs left".format(timeRemainingSec),
            )
        }
        if (ballsHeld > 0 && timeRemainingSec <= scoreTime + PLANNER_ENDGAME_MARGIN_SEC) {
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", 0.0, 0.0,
                ballsHeld * POLLEN_VALUE, scoreTime,
                "ENDGAME score run: %.0fs left, need %.1fs".format(timeRemainingSec, scoreTime),
            )
        }

        // ---- 2. FULL: no point holding more than we can carry ----
        if (ballsHeld >= MAX_BALLS_HELD && hive != null) {
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", hive.first, hive.second,
                ballsHeld * POLLEN_VALUE, scoreTime,
                "FULL (%d balls) -> score".format(ballsHeld),
            )
        }

        // ---- 3. Evaluate collect candidates (balls, then flowers) ----
        val opponents = world.opponents()
        var bestBall: TrackedObject? = null
        var bestDensity = 0.0
        var bestTime = 0.0
        var bestEffValue = 0.0
        for (b in world.balls()) {
            val dist = b.distanceTo(robotXcm, robotYcm)
            val t = dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_COLLECT_HANDLE_SEC
            var v = b.value * b.confidence
            // opponent closer to this ball than we are? they'll likely take it
            for (o in opponents) {
                if (o.distanceTo(b.fieldXcm, b.fieldYcm) < dist) {
                    v *= PLANNER_OPPONENT_DISCOUNT
                    break
                }
            }
            // can't collect AND still score it afterwards?
            if (timeRemainingSec < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
            val density = v / (1.0 + t)
            if (density > bestDensity) {
                bestDensity = density; bestBall = b; bestTime = t; bestEffValue = v
            }
        }

        var bestFlower: TrackedObject? = null
        var flowerDensity = 0.0
        var flowerTime = 0.0
        if (FLOWER_PULL_ENABLED && ballsHeld < MAX_BALLS_HELD) {
            for (fl in world.flowers()) {
                val dist = fl.distanceTo(robotXcm, robotYcm)
                val t = dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_FLOWER_TOTAL_SEC
                val v = POLLEN_VALUE * fl.confidence
                if (timeRemainingSec < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
                val density = v / (1.0 + t)
                if (density > flowerDensity) {
                    flowerDensity = density; bestFlower = fl; flowerTime = t
                }
            }
        }

        // Scoring what we hold is always an option; compare value densities.
        val scoreDensity =
            if (ballsHeld > 0) (ballsHeld * POLLEN_VALUE) / (1.0 + scoreTime) else 0.0

        // ---- 4. Choose ----
        val flowerWins =
            bestFlower != null && flowerDensity > bestDensity && flowerDensity > scoreDensity
        val ballWins = bestBall != null && bestDensity > scoreDensity
        if (ballsHeld > 0 && !ballWins && !flowerWins && hive != null) {
            return PlannerDecision(
                PlannerAction.GO_SCORE, -1, "Hive", hive.first, hive.second,
                ballsHeld * POLLEN_VALUE, scoreTime,
                "score beats collecting (%.2f vs %.2f density)".format(scoreDensity, bestDensity),
            )
        }
        if (flowerWins) {
            return PlannerDecision(
                PlannerAction.USE_FLOWER, bestFlower!!.id, bestFlower.label,
                bestFlower.fieldXcm, bestFlower.fieldYcm, POLLEN_VALUE, flowerTime,
                "best density %.3f (flower)".format(flowerDensity),
            )
        }
        if (ballWins || (bestBall != null && ballsHeld == 0)) {
            return PlannerDecision(
                PlannerAction.COLLECT_BALL, bestBall!!.id, bestBall.label,
                bestBall.fieldXcm, bestBall.fieldYcm, bestEffValue, bestTime,
                "best density %.3f (val %.1f conf %.0f%%)".format(
                    bestDensity, bestBall.value, bestBall.confidence * 100,
                ),
            )
        }
        return PlannerDecision(
            PlannerAction.HOLD, -1, "", 0.0, 0.0, 0.0, 0.0,
            "nothing worth doing — hold & scan",
        )
    }

    /** Logs a decision to the Black Box with a source tag (real / shadow). */
    fun logDecision(d: PlannerDecision, source: String, ballsHeld: Int, timeRemainingSec: Double) {
        BlackBox.event(
            "planner_decision",
            "source" to source,
            "action" to d.action.name,
            "targetId" to d.targetId,
            "targetLabel" to d.targetLabel,
            "value" to d.expectedValue,
            "costSec" to d.estCostSec,
            "ballsHeld" to ballsHeld,
            "timeRemainingSec" to timeRemainingSec,
            "why" to d.rationale,
        )
    }
}