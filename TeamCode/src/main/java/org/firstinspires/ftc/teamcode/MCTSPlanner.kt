package org.firstinspires.ftc.teamcode

import java.util.Locale
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.random.Random

/**
 * ============ MCTS PLANNER — Monte Carlo Tree Search for deeper lookahead ============
 *
 * Runs a limited number of playouts (default [MCTS_PLAYOUTS]) from the
 * current WorldModel state to evaluate action sequences. Each playout
 * simulates stochastic outcomes (pickup success/failure, opponent
 * interference, vision loss) and accumulates discounted value. The action
 * with the highest average return is chosen.
 *
 * This replaces the greedy one-step value-density with multi-step lookahead,
 * naturally handling:
 *   - Multi-ball sequencing (which ball first to minimize total travel)
 *   - Flower vs ball tradeoffs over the full remaining time
 *   - Endgame risk (when to commit to scoring vs collecting one more)
 *   - Opponent contest modeling (stochastic discounting)
 *
 * Two refinements make the search much stronger than plain UCT:
 *
 *  * PROGRESSIVE WIDENING — the number of children a node may hold grows
 *    with its visit count (`1 + floor(k * sqrt(visits))`). Without it a node
 *    with 20 candidate actions explodes the tree and 128 playouts never get
 *    past the first level; with it the tree stays deep AND wide where it
 *    matters.
 *
 *  * HEURISTIC-GUIDED PLAYOUTS — a playout step follows the
 *    [StrategicPlanner]'s pick most of the time and otherwise samples by
 *    value density. Pure random playouts spend almost all of their budget
 *    on hopeless sequences (driving to a ball the opponent already took);
 *    following the heuristic puts the random rollouts in the region of the
 *    tree where the good decisions live, so the same 128 playouts buy far
 *    more lookahead.
 *
 * Pure Kotlin, no allocations in the hot path (reuses arrays), runs in a few
 * milliseconds on the Robot Controller.
 */
class MCTSPlanner(
    val playouts: Int = MCTS_PLAYOUTS,
    val maxDepth: Int = MCTS_MAX_DEPTH,
    /** Progressive-widening coefficient (children = 1 + floor(k*sqrt(visits))). */
    val progressiveWideningK: Double = MCTS_PROGRESSIVE_WIDENING_K,
    /** Probability that a playout step follows the StrategicPlanner's pick. */
    val heuristicWeight: Double = MCTS_HEURISTIC_WEIGHT,
    /** Wall-clock budget for one decide() call, milliseconds. */
    val timeBudgetMs: Double = MCTS_TIME_BUDGET_MS,
    private val random: Random = Random.Default,
) {

    /** Playouts actually completed by the last [decide] call. */
    var lastPlayoutsDone: Int = 0
        private set

    /** Milliseconds spent in the last [decide] call. */
    var lastSearchMillis: Double = 0.0
        private set

    /** Number of nodes created by the last [decide] call. */
    var lastNodeCount: Int = 0
        private set

    /** Reused heuristic policy (avoid re-allocating a planner per playout). */
    private val heuristic = StrategicPlanner()

    /** MCTS node representing a state in the search tree. */
    private class Node(
        val state: SimState,
        val parent: Node? = null,
        val action: PlannerAction? = null,
        val targetId: Int = -1,
        /** Discounted immediate rewards earned on the root-to-node path. */
        val pathValue: Double = 0.0,
        val depth: Int = 0,
    ) {
        var visits = 0
        var totalValue = 0.0
        var children: Array<Node?> = emptyArray()
        var untriedActions: List<PlannerDecision> = emptyList()

        val world: WorldModel get() = state.world
        val ballsHeld: Int get() = state.ballsHeld
        val timeRemaining: Double get() = state.timeRemaining
        val robotX: Double get() = state.robotX
        val robotY: Double get() = state.robotY

        fun ucb1(exploration: Double = 1.414): Double {
            if (visits == 0) return Double.POSITIVE_INFINITY
            val parentVisits = (parent?.visits ?: 1).toDouble()
            val visitsD = visits.toDouble()
            return totalValue / visitsD + exploration * Math.sqrt(Math.log(parentVisits) / visitsD)
        }
    }

    /**
     * State for a single playout simulation.
     *
     * The "already collected" and "already pulled" bookkeeping lives HERE
     * rather than in the WorldModel. A playout is a HYPOTHETICAL future: if
     * it were allowed to call `world.removeBall()` or mark a real flower
     * cluster depleted, 128 playouts per planner tick would delete real
     * balls out of the robot's actual map and black out real flower patches
     * it had never touched. The sets below are the simulation's private
     * overlay, so the live world is never mutated by the search.
     *
     * Exposed ([internal]) so unit tests can build a simulation state and
     * assert that failed outcomes leave it untouched.
     */
    internal class SimState(
        var world: WorldModel,
        var ballsHeld: Int,
        var timeRemaining: Double,
        var robotX: Double,
        var robotY: Double,
        /** Ball track ids the playout has already picked up. */
        val collectedBallIds: MutableSet<Int> = mutableSetOf(),
        /** Flower track ids the playout has already pulled. */
        val pulledFlowerIds: MutableSet<Int> = mutableSetOf(),
        /** Flower cluster ids the playout has already pulled. */
        val pulledClusterIds: MutableSet<Int> = mutableSetOf(),
        /** Full score value of the held contents, including nectar flowers. */
        var heldScoreValue: Double = ballsHeld * POLLEN_VALUE,
    ) {
        /** Deep-ish copy: the overlay sets are copied so branches stay independent. */
        fun copy(): SimState = SimState(
            world = world,
            ballsHeld = ballsHeld,
            timeRemaining = timeRemaining,
            robotX = robotX,
            robotY = robotY,
            collectedBallIds = collectedBallIds.toMutableSet(),
            pulledFlowerIds = pulledFlowerIds.toMutableSet(),
            pulledClusterIds = pulledClusterIds.toMutableSet(),
            heldScoreValue = heldScoreValue,
        )
    }

    /**
     * Runs MCTS from the current state and returns the best action.
     * Falls back to greedy planner if MCTS fails or times out.
     */
    fun decide(
        world: WorldModel,
        ballsHeld: Int,
        timeRemainingSec: Double,
        robotXcm: Double,
        robotYcm: Double,
    ): PlannerDecision {
        val startNanos = System.nanoTime()
        val timeBudgetNanos = (timeBudgetMs * 1_000_000.0).toLong()

        // Quick greedy evaluation for fallback and initial child values
        val greedyDecision = heuristic.decide(world, ballsHeld, timeRemainingSec, robotXcm, robotYcm)

        // Build root node
        val rootState = SimState(world, ballsHeld, timeRemainingSec, robotXcm, robotYcm)
        val root = Node(rootState)
        root.untriedActions = generateActions(rootState)

        var playoutsDone = 0
        while (playoutsDone < playouts && System.nanoTime() - startNanos < timeBudgetNanos) {
            val node = select(root)
            val value = simulate(node)
            backpropagate(node, value)
            playoutsDone++
        }

        lastPlayoutsDone = playoutsDone
        lastSearchMillis = (System.nanoTime() - startNanos) / 1e6
        lastNodeCount = countNodes(root)

        // Pick best child by average value (or visits for stability)
        val bestChild = root.children
            .filterNotNull()
            .maxByOrNull { it.visits } // most visited = most promising
            ?: return greedyDecision // fallback

        return PlannerDecision(
            bestChild.action!!,
            bestChild.targetId,
            bestChild.world.trackById(bestChild.targetId)?.label ?: "",
            bestChild.world.trackById(bestChild.targetId)?.fieldXcm ?: 0.0,
            bestChild.world.trackById(bestChild.targetId)?.fieldYcm ?: 0.0,
            bestChild.totalValue / bestChild.visits,
            estimateCost(bestChild.action!!, bestChild.targetId, bestChild.world, robotXcm, robotYcm),
            "MCTS: %d playouts, %d nodes, %.1fms, avg value %.2f".format(
                playoutsDone, lastNodeCount, lastSearchMillis, bestChild.totalValue / bestChild.visits,
            ),
        )
    }

    /** Counts the nodes in the tree (diagnostics / tests). */
    private fun countNodes(node: Node): Int {
        var n = 1
        for (c in node.children) if (c != null) n += countNodes(c)
        return n
    }

    /**
     * The maximum number of children a node may hold, given its visit count.
     *
     * Progressive widening: `1 + floor(k * sqrt(visits))`. A fresh node
     * gets one child; a node that has been visited 100 times may hold 15.
     * This keeps the search's branching factor bounded while still letting a
     * proven branch explore its alternatives.
     */
    private fun maxChildrenFor(node: Node): Int = maxChildrenForVisits(node.visits)

    /**
     * The progressive-widening limit for a node with [visits] visits.
     *
     * Exposed so the formula can be pinned by a unit test — it is the
     * single knob that decides how deep and how wide the tree gets.
     */
    fun maxChildrenForVisits(visits: Int): Int {
        val limit = 1 + Math.floor(progressiveWideningK * Math.sqrt(visits.toDouble())).toInt()
        return max(1, limit)
    }

    /** Selection: traverse tree using UCB1 until leaf or unexpanded node. */
    private fun select(node: Node): Node {
        var current = node
        while (current.untriedActions.isEmpty() && current.children.any { it != null }) {
            current = current.children
                .filterNotNull()
                .maxByOrNull { it.ucb1() }
                ?: return current
        }
        if (current.untriedActions.isNotEmpty() && current.children.size < maxChildrenFor(current)) {
            return expand(current)
        }
        return current
    }

    /** Expansion: create a child node for a random untried action. */
    private fun expand(node: Node): Node {
        if (node.children.size >= maxChildrenFor(node)) return node
        val untried = node.untriedActions.toMutableList()
        if (untried.isEmpty()) return node
        val action = untried.removeAt(random.nextInt(untried.size))
        node.untriedActions = untried
        // Resolve the outcome once so expansion is coherent with rollouts:
        // the same draw that decides success also drives the state transition.
        val outcome = resolveAction(action, node.state)
        val newState = applyAction(action, node.state, outcome)
        val child = Node(
            state = newState,
            parent = node,
            action = action.action,
            targetId = action.targetId,
            pathValue = node.pathValue + outcome.immediateValue * Math.pow(0.95, node.depth.toDouble()),
            depth = node.depth + 1,
        )
        child.untriedActions = generateActions(newState)
        // Add to parent's children array
        node.children = (node.children.toMutableList() + child).toTypedArray()
        return child
    }

    /**
     * Simulation: guided playout to terminal state.
     *
     * Each step picks an action with [selectPlayoutAction] — the
     * StrategicPlanner's pick most of the time, value-density sampling the
     * rest — so the random rollouts explore the neighbourhood of good play
     * instead of wasting the budget on hopeless sequences.
     */
    private fun simulate(node: Node): Double {
        // Copy the node's state so the playout's own collection/pulling
        // bookkeeping does not leak back into the tree.
        var state = node.state.copy()
        // Expansion resolves real rewards too. Carry the whole root-to-node
        // path or a successful expanded GO_SCORE would look worse than HOLD.
        var totalValue = node.pathValue
        var depth = node.depth

        while (depth < maxDepth && state.timeRemaining > PLANNER_PARK_SEC + 1.0) {
            val actions = generateActions(state)
            if (actions.isEmpty()) break

            val action = selectPlayoutAction(state, actions)
            // Resolve the outcome once: the immediate reward AND the state
            // transition come from the same draw, so a failed pickup never
            // puts a ball in the hopper and a failed score never empties it.
            val outcome = resolveAction(action, state)
            totalValue += outcome.immediateValue * Math.pow(0.95, depth.toDouble()) // discount future
            state = applyAction(action, state, outcome)
            depth++

            if (state.timeRemaining <= PLANNER_PARK_SEC + 1.0) {
                // Forced park
                totalValue += 0.0 // park has no direct value
                break
            }
        }

        // Terminal value: partial credit for the exact held contents.
        totalValue += state.heldScoreValue * 0.5
        return totalValue
    }

    /**
     * Picks the action a playout step should take.
     *
     * With probability [heuristicWeight] we follow the StrategicPlanner's
     * own decision (policy guidance) — but only when that decision is one
     * of the legal actions here, since the planner and the candidate
     * generator do not have exactly the same admissibility rules.
     *
     * Otherwise we sample by value density (`value / (1 + cost)` — the same
     * trade the StrategicPlanner makes) with a softmax, so the search still
     * explores alternatives the heuristic dislikes.
     */
    private fun selectPlayoutAction(state: SimState, actions: List<PlannerDecision>): PlannerDecision {
        if (actions.size == 1) return actions[0]

        if (random.nextDouble() < heuristicWeight) {
            val h = heuristic.decide(
                state.world, state.ballsHeld, state.timeRemaining, state.robotX, state.robotY,
            )
            for (a in actions) {
                if (a.action == h.action && a.targetId == h.targetId) return a
            }
        }
        return sampleByDensity(actions)
    }

    /**
     * Samples an action proportionally to a softmax over its value density.
     * Normalised by the best density so the weights stay in a sane range.
     */
    private fun sampleByDensity(actions: List<PlannerDecision>): PlannerDecision {
        var maxDensity = 0.0
        for (a in actions) {
            val d = a.expectedValue / (1.0 + a.estCostSec)
            if (d > maxDensity) maxDensity = d
        }
        if (maxDensity <= 0.0) return actions[random.nextInt(actions.size)]

        var total = 0.0
        val weights = DoubleArray(actions.size)
        for (i in actions.indices) {
            val a = actions[i]
            val density = a.expectedValue / (1.0 + a.estCostSec)
            val norm = (density / maxDensity).coerceIn(0.0, 1.0)
            val w = exp((norm - 1.0) * MCTS_SOFTMAX_TEMPERATURE)
            weights[i] = w
            total += w
        }
        if (total <= 0.0) return actions[random.nextInt(actions.size)]

        var r = random.nextDouble() * total
        for (i in actions.indices) {
            r -= weights[i]
            if (r <= 0.0) return actions[i]
        }
        return actions[actions.size - 1]
    }

    /** Backpropagation: update visit counts and values up the tree. */
    private fun backpropagate(node: Node?, value: Double) {
        var current = node
        while (current != null) {
            current.visits++
            current.totalValue += value
            current = current.parent
        }
    }

    /**
     * Generates all legal actions from the current state.
     *
     * [state] carries the simulation's private overlay, so a ball the
     * playout already picked up (or a flower patch it already emptied) is
     * not offered again — without that the playout would keep re-collecting
     * the same ball and the search would believe in infinite pollen.
     */
    internal fun generateActions(state: SimState): List<PlannerDecision> {
        val world = state.world
        val ballsHeld = state.ballsHeld
        val timeRemaining = state.timeRemaining
        val robotX = state.robotX
        val robotY = state.robotY
        val actions = mutableListOf<PlannerDecision>()
        val hive = world.hive()
        val distToHive = if (hive != null) hypot(hive.first - robotX, hive.second - robotY) else 150.0
        val baseScoreTime = distToHive / ROBOT_SPEED_CM_PER_SEC
        val hiveObstaclePenalty = if (hive != null && world.isPathBlocked(robotX, robotY, hive.first, hive.second)) 3.0 else 0.0
        val scoreTime = baseScoreTime + PLANNER_SCORE_TOTAL_SEC + hiveObstaclePenalty
        val opponents = world.opponents()

        // PARK if time is critical
        if (timeRemaining <= PLANNER_PARK_SEC + 2.0) {
            actions.add(PlannerDecision(PlannerAction.PARK, -1, "", 0.0, 0.0, 0.0, PLANNER_PARK_SEC, "MCTS park"))
            return actions
        }

        // GO_SCORE if holding balls and time is tight or full
        if (ballsHeld > 0 && (timeRemaining <= scoreTime + PLANNER_ENDGAME_MARGIN_SEC || ballsHeld >= MAX_BALLS_HELD)) {
            actions.add(PlannerDecision(PlannerAction.GO_SCORE, -1, "Hive", hive?.first ?: 0.0, hive?.second ?: 0.0, ballsHeld * POLLEN_VALUE, scoreTime, "MCTS score"))
        }

        // COLLECT_BALL for each viable ball (never when the hopper is full)
        if (state.ballsHeld < MAX_BALLS_HELD) {
            for (b in world.balls()) {
                if (b.id in state.collectedBallIds) continue // already picked up in this playout
                if (b.ageSec(System.nanoTime()) > DEAD_RECKON_MAX_AGE_SEC) continue
                if (b.lastAreaPx < CHASE_MIN_CONTOUR_AREA_PX) continue
                val dist = b.distanceTo(robotX, robotY)
                val baseT = dist / ROBOT_SPEED_CM_PER_SEC
                val obstaclePenalty = if (world.isPathBlocked(robotX, robotY, b.fieldXcm, b.fieldYcm)) 2.0 else 0.0
                val t = baseT + PLANNER_COLLECT_HANDLE_SEC + obstaclePenalty
                if (timeRemaining < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
                var v = b.value * b.confidence
                for (o in opponents) {
                    if (o.distanceTo(b.fieldXcm, b.fieldYcm) < dist) {
                        val intentDiscount = when (o.intent) {
                            OpponentIntent.DEFENDING -> 0.3
                            OpponentIntent.SCORING -> 0.5
                            OpponentIntent.COLLECTING -> 0.4
                            OpponentIntent.TRANSIT -> 0.7
                            OpponentIntent.UNKNOWN -> PLANNER_OPPONENT_DISCOUNT
                        }
                        v *= intentDiscount * o.intentConfidence.coerceIn(0.3, 1.0) * (0.8 + 0.4 * random.nextDouble())
                        break
                    }
                }
                actions.add(PlannerDecision(PlannerAction.COLLECT_BALL, b.id, b.label, b.fieldXcm, b.fieldYcm, v, t, "MCTS collect"))
            }
        }

        // USE_FLOWER for each viable flower cluster
        if (FLOWER_PULL_ENABLED && ballsHeld < MAX_BALLS_HELD) {
            val now = System.nanoTime()
            val liveClusters = world.availableFlowerClusters(now).filter { it.id !in state.pulledClusterIds }
            // Evaluate flower clusters (multi-flower nectar yield). Only
            // clusters that are FULL and hold usable flowers are considered.
            for (cluster in liveClusters) {
                val usable = cluster.flowers.count {
                    it.id !in state.pulledFlowerIds && world.flowerState(it.id, now) == FlowerState.FULL
                }
                if (usable <= 0) continue
                val dist = cluster.distanceTo(robotX, robotY)
                val baseT = dist / ROBOT_SPEED_CM_PER_SEC
                val obstaclePenalty = if (world.isPathBlocked(robotX, robotY, cluster.centerXcm, cluster.centerYcm)) 3.0 else 0.0
                val t = baseT + PLANNER_FLOWER_TOTAL_SEC * usable + obstaclePenalty
                if (timeRemaining < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
                if (ballsHeld + usable > MAX_BALLS_HELD) continue
                val v = usable * NECTAR_VALUE * (0.85 + 0.3 * random.nextDouble()) // stochastic nectar yield
                actions.add(PlannerDecision(PlannerAction.USE_FLOWER, cluster.id, "FlowerCluster", cluster.centerXcm, cluster.centerYcm, v, t, "MCTS flower cluster"))
            }
            // Fallback to individual flowers if no clusters
            if (liveClusters.isEmpty()) {
                for (fl in world.availableFlowers(now)) {
                    if (fl.id in state.pulledFlowerIds) continue
                    val dist = fl.distanceTo(robotX, robotY)
                    val baseT = dist / ROBOT_SPEED_CM_PER_SEC
                    val obstaclePenalty = if (world.isPathBlocked(robotX, robotY, fl.fieldXcm, fl.fieldYcm)) 2.0 else 0.0
                    val t = baseT + PLANNER_FLOWER_TOTAL_SEC + obstaclePenalty
                    if (timeRemaining < t + scoreTime + PLANNER_ENDGAME_MARGIN_SEC) continue
                    val v = NECTAR_VALUE * fl.confidence * (0.9 + 0.2 * random.nextDouble())
                    actions.add(PlannerDecision(PlannerAction.USE_FLOWER, fl.id, fl.label, fl.fieldXcm, fl.fieldYcm, v, t, "MCTS flower"))
                }
            }
        }

        // GO_SCORE as voluntary option (not forced)
        if (ballsHeld > 0 && hive != null) {
            actions.add(PlannerDecision(PlannerAction.GO_SCORE, -1, "Hive", hive.first, hive.second, ballsHeld * POLLEN_VALUE, scoreTime, "MCTS voluntary score"))
        }

        // HOLD (scan)
        actions.add(PlannerDecision(PlannerAction.HOLD, -1, "", 0.0, 0.0, 0.0, 0.5, "MCTS hold"))

        return actions
    }

    /**
     * The realized outcome of an action in the simulation: whether it
     * succeeded, the immediate reward it yields, and the time it costs.
     *
     * The outcome is resolved ONCE ([resolveAction]) and reused by both value
     * estimation ([simulate]) and state transition ([applyAction]), so the
     * two stay coherent: a pickup that "failed" never puts a ball in the
     * hopper, and a score that "failed" never empties it.
     *
     * Value is awarded exactly once. Collection itself yields no immediate
     * points — the held contents are paid at [GO_SCORE] using [SimState.heldScoreValue]
     * (including nectar value) or as partial terminal credit. This prevents a
     * pickup from being rewarded both at collection and at scoring/terminal.
     */
    internal data class ActionOutcome(
        val succeeded: Boolean,
        val immediateValue: Double,
        val cost: Double,
    )

    /** Resolves the stochastic outcome of [action] from [state]. */
    internal fun resolveAction(action: PlannerDecision, state: SimState): ActionOutcome {
        val (succeeded, value) = when (action.action) {
            PlannerAction.COLLECT_BALL -> {
                // Stochastic pickup success (80-95% based on confidence)
                val track = state.world.trackById(action.targetId)
                val successProb = (track?.confidence ?: 0.5) * 0.9 + 0.1
                val success = random.nextDouble() < successProb
                // Opponent might steal during approach
                val stolen = state.world.opponents().any { o ->
                    o.distanceTo(action.targetFieldXcm, action.targetFieldYcm) < 50.0 && random.nextDouble() < 0.3
                }
                // No immediate points at collection: value is awarded once at
                // scoring (GO_SCORE) or as partial terminal credit.
                Pair(success && !stolen, 0.0)
            }
            PlannerAction.GO_SCORE -> {
                // Stochastic scoring success (90% base, drops with distance)
                val hive = state.world.hive()
                val dist = if (hive != null) hypot(hive.first - state.robotX, hive.second - state.robotY) else 150.0
                val scoreProb = (0.95 - (dist / 200.0) * 0.2).coerceIn(0.5, 0.95)
                val success = random.nextDouble() < scoreProb
                Pair(success, if (success) state.heldScoreValue else 0.0)
            }
            PlannerAction.USE_FLOWER -> {
                val success = random.nextDouble() < 0.85
                // Like ball collection, a flower adds future scoring value to
                // the hopper; it is not also an immediate score.
                Pair(success, 0.0)
            }
            PlannerAction.PARK -> Pair(true, 0.0)
            PlannerAction.HOLD -> Pair(true, 0.0)
        }
        val cost = when (action.action) {
            PlannerAction.PARK -> PLANNER_PARK_SEC
            PlannerAction.HOLD -> 0.5
            else -> action.estCostSec
        }
        return ActionOutcome(succeeded, value, cost)
    }

    /**
     * Applies an action to the simulation state, consistent with [outcome]
     * as resolved by [resolveAction]. A failed pickup adds nothing to the
     * hopper and marks no track as collected; a failed score empties nothing.
     *
     * The returned state is a COPY: the live WorldModel is never touched by
     * the search. Balls "collected" and flowers "pulled" during the playout
     * are recorded in the simulation overlay (see [SimState]) so the playout
     * cannot pick the same ball up twice.
     */
    internal fun applyAction(action: PlannerDecision, state: SimState, outcome: ActionOutcome): SimState {
        val newState = state.copy()
        newState.timeRemaining -= outcome.cost
        when (action.action) {
            PlannerAction.COLLECT_BALL -> {
                // Only a successful pickup puts a ball in the hopper and
                // records the track as collected.
                if (outcome.succeeded && newState.ballsHeld < MAX_BALLS_HELD) {
                    newState.ballsHeld++
                    newState.heldScoreValue += newState.world.trackById(action.targetId)?.value ?: POLLEN_VALUE
                    newState.collectedBallIds.add(action.targetId)
                }
                newState.robotX = action.targetFieldXcm
                newState.robotY = action.targetFieldYcm
            }
            PlannerAction.GO_SCORE -> {
                if (outcome.succeeded) {
                    newState.ballsHeld = 0 // scored all
                    newState.heldScoreValue = 0.0
                }
                val hive = newState.world.hive()
                if (hive != null) {
                    newState.robotX = hive.first
                    newState.robotY = hive.second
                }
            }
            PlannerAction.USE_FLOWER -> {
                if (outcome.succeeded) {
                    // A cluster id means "pull this patch"; anything else is a
                    // single flower.
                    val cluster = newState.world.flowerClusters().firstOrNull { it.id == action.targetId }
                    if (cluster != null) {
                        val now = System.nanoTime()
                        val usable = cluster.flowers.count {
                            it.id !in newState.pulledFlowerIds &&
                                newState.world.flowerState(it.id, now) == FlowerState.FULL
                        }
                        val gained = minOf(usable, MAX_BALLS_HELD - newState.ballsHeld)
                        newState.ballsHeld += gained
                        if (gained > 0) newState.heldScoreValue += action.expectedValue
                        newState.pulledClusterIds.add(cluster.id)
                        for (f in cluster.flowers) newState.pulledFlowerIds.add(f.id)
                        newState.robotX = cluster.centerXcm
                        newState.robotY = cluster.centerYcm
                    } else {
                        if (newState.ballsHeld < MAX_BALLS_HELD) {
                            newState.ballsHeld++
                            newState.heldScoreValue += action.expectedValue
                        }
                        newState.pulledFlowerIds.add(action.targetId)
                        newState.robotX = action.targetFieldXcm
                        newState.robotY = action.targetFieldYcm
                    }
                }
            }
            PlannerAction.PARK -> {}
            PlannerAction.HOLD -> {}
        }
        return newState
    }

    private fun estimateCost(action: PlannerAction, targetId: Int, world: WorldModel, robotX: Double, robotY: Double): Double {
        return when (action) {
            PlannerAction.COLLECT_BALL -> {
                val track = world.trackById(targetId)
                if (track == null) return PLANNER_COLLECT_HANDLE_SEC
                val dist = track.distanceTo(robotX, robotY)
                dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_COLLECT_HANDLE_SEC
            }
            PlannerAction.GO_SCORE -> {
                val hive = world.hive()
                if (hive == null) return PLANNER_SCORE_TOTAL_SEC
                val dist = hypot(hive.first - robotX, hive.second - robotY)
                dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_SCORE_TOTAL_SEC
            }
            PlannerAction.USE_FLOWER -> {
                val cluster = world.flowerClusters().firstOrNull { it.id == targetId }
                if (cluster != null) {
                    val dist = cluster.distanceTo(robotX, robotY)
                    dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_FLOWER_TOTAL_SEC * cluster.flowerCount
                } else {
                    val track = world.trackById(targetId)
                    if (track == null) return PLANNER_FLOWER_TOTAL_SEC
                    val dist = track.distanceTo(robotX, robotY)
                    dist / ROBOT_SPEED_CM_PER_SEC + PLANNER_FLOWER_TOTAL_SEC
                }
            }
            PlannerAction.PARK -> PLANNER_PARK_SEC
            PlannerAction.HOLD -> 0.5
        }
    }

    /** Logs a decision to the Black Box with a source tag (real / shadow). */
    fun logDecision(d: PlannerDecision, source: String, ballsHeld: Int, timeRemainingSec: Double) {
        StrategicPlanner().logDecision(d, source, ballsHeld, timeRemainingSec)
    }
}