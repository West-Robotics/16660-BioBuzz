package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Twist2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.DrivetrainBackLeft
import org.firstinspires.ftc.teamcode.DrivetrainBackRight
import org.firstinspires.ftc.teamcode.DrivetrainFrontLeft
import org.firstinspires.ftc.teamcode.DrivetrainFrontRight
import org.firstinspires.ftc.teamcode.RR_IMU_NAME
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage
import java.util.LinkedList
import kotlin.math.ceil
import kotlin.math.max

/**
 * Road Runner 1.0 mecanum drive, ported to Kotlin from the official quickstart
 * (https://github.com/acmerobotics/road-runner-quickstart) to match this
 * project's Kotlin + NextFTC architecture. Units are INCHES and RADIANS
 * throughout Road Runner (per RR 1.0 convention), NOT centimeters.
 */
@Config
class MecanumDrive(hardwareMap: HardwareMap, initialPose: Pose2d) {

    /** Tunable parameters (editable live via FTC Dashboard once it is started). */
    class Params {
        // IMU orientation
        // TODO: fill in these values based on the physical Control/Expansion Hub mounting.
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
        var logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP
        var usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD

        // drive model parameters
        var inPerTick = 1.0
        var lateralInPerTick = inPerTick
        var trackWidthTicks = 0.0

        // feedforward parameters (in tick units)
        var kS = 0.0
        var kV = 0.0
        var kA = 0.0

        // path profile parameters (in inches)
        var maxWheelVel = 50.0
        var minProfileAccel = -30.0
        var maxProfileAccel = 50.0

        // turn profile parameters (in radians)
        var maxAngVel = Math.PI // shared with path
        var maxAngAccel = Math.PI

        // path controller gains
        var axialGain = 0.0
        var lateralGain = 0.0
        var headingGain = 0.0 // shared with turn

        var axialVelGain = 0.0
        var lateralVelGain = 0.0
        var headingVelGain = 0.0 // shared with turn
    }

    val kinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks,
        PARAMS.inPerTick / PARAMS.lateralInPerTick,
    )

    val defaultTurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel,
    )
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel),
        ),
    )
    val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    val leftFront: DcMotorEx
    val leftBack: DcMotorEx
    val rightBack: DcMotorEx
    val rightFront: DcMotorEx

    val voltageSensor: VoltageSensor

    val lazyImu: com.acmerobotics.roadrunner.ftc.LazyImu

    /** Pluggable localization; defaults to mecanum + IMU. Swappable for dead wheels / Pinpoint. */
    var localizer: Localizer

    /**
     * Optional external pose correction hook (the hybrid visual localization
     * system attaches here). See [PoseCorrector]. Null = pure Road Runner.
     */
    var poseCorrector: PoseCorrector? = null

    /**
     * Master safety gate: vision corrections are only allowed to move the Road
     * Runner pose when this is true AND a [PoseCorrector] is attached. Keep it
     * false until the visual test procedure (see VisionConfig.kt docs) has been
     * passed on the physical robot.
     */
    var visionCorrectionEnabled = false

    private val poseHistory = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50_000_000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50_000_000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50_000_000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50_000_000)

    /** Mecanum + IMU localizer (Road Runner 1.0 quickstart algorithm, Kotlin port). */
    inner class DriveLocalizer(initialPose: Pose2d) : Localizer {
        val leftFront = OverflowEncoder(RawEncoder(this@MecanumDrive.leftFront))
        val leftBack = OverflowEncoder(RawEncoder(this@MecanumDrive.leftBack))
        val rightBack = OverflowEncoder(RawEncoder(this@MecanumDrive.rightBack))
        val rightFront = OverflowEncoder(RawEncoder(this@MecanumDrive.rightFront))
        val imu: IMU = lazyImu.get()

        private var lastLeftFrontPos = 0
        private var lastLeftBackPos = 0
        private var lastRightBackPos = 0
        private var lastRightFrontPos = 0
        private var lastHeading = Rotation2d.exp(0.0)
        private var initialized = false
        private var pose = initialPose

        override fun setPose(pose: Pose2d) {
            this.pose = pose
        }

        override fun getPose(): Pose2d = pose

        override fun update(): PoseVelocity2d? {
            val leftFrontPosVel = leftFront.getPositionAndVelocity()
            val leftBackPosVel = leftBack.getPositionAndVelocity()
            val rightBackPosVel = rightBack.getPositionAndVelocity()
            val rightFrontPosVel = rightFront.getPositionAndVelocity()

            val angles: YawPitchRollAngles = imu.robotYawPitchRollAngles

            FlightRecorder.write(
                "MECANUM_LOCALIZER_INPUTS",
                MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles,
                ),
            )

            val heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

            if (!initialized) {
                initialized = true

                lastLeftFrontPos = leftFrontPosVel.position
                lastLeftBackPos = leftBackPosVel.position
                lastRightBackPos = rightBackPosVel.position
                lastRightFrontPos = rightFrontPosVel.position

                lastHeading = heading

                return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            }

            val headingDelta = heading.minus(lastHeading)
            val twist: Twist2dDual<Time> = kinematics.forward(
                MecanumKinematics.WheelIncrements(
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftFrontPosVel.position - lastLeftFrontPos).toDouble(),
                            (leftFrontPosVel.velocity ?: 0).toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftBackPosVel.position - lastLeftBackPos).toDouble(),
                            (leftBackPosVel.velocity ?: 0).toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightBackPosVel.position - lastRightBackPos).toDouble(),
                            (rightBackPosVel.velocity ?: 0).toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightFrontPosVel.position - lastRightFrontPos).toDouble(),
                            (rightFrontPosVel.velocity ?: 0).toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                ),
            )

            lastLeftFrontPos = leftFrontPosVel.position
            lastLeftBackPos = leftBackPosVel.position
            lastRightBackPos = rightBackPosVel.position
            lastRightFrontPos = rightFrontPosVel.position

            lastHeading = heading

            pose = pose.plus(Twist2d(twist.line.value(), headingDelta))

            return twist.velocity().value()
        }
    }

    init {
        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // Motor names come from the existing hardware configuration (globals.kt).
        leftFront = hardwareMap.get(DcMotorEx::class.java, DrivetrainFrontLeft)
        leftBack = hardwareMap.get(DcMotorEx::class.java, DrivetrainBackLeft)
        rightBack = hardwareMap.get(DcMotorEx::class.java, DrivetrainBackRight)
        rightFront = hardwareMap.get(DcMotorEx::class.java, DrivetrainFrontRight)

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: reverse motor directions if needed
        //   leftFront.direction = DcMotorSimple.Direction.REVERSE

        lazyImu = LazyHardwareMapImu(
            hardwareMap,
            RR_IMU_NAME,
            RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection),
        )

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        localizer = DriveLocalizer(initialPose)

        FlightRecorder.write("MECANUM_PARAMS", PARAMS)
    }

    var pose: Pose2d
        get() = localizer.getPose()
        set(value) = localizer.setPose(value)

    /**
     * Updates the pose estimate. This is the *only* integration point for visual
     * corrections: the [PoseCorrector] (if attached and enabled) is consulted
     * here, and its (already clamped and confidence-gated) corrected pose is
     * written back into the localizer. Road Runner pose estimation is never
     * bypassed — it is merely nudged.
     */
    fun updatePoseEstimate(): PoseVelocity2d? {
        val vel = localizer.update()

        val corrector = poseCorrector
        if (corrector != null && visionCorrectionEnabled) {
            val corrected = corrector.correct(localizer.getPose(), vel, System.nanoTime())
            localizer.setPose(corrected)
        }

        poseHistory.add(localizer.getPose())
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(localizer.getPose()))

        return vel
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels = MecanumKinematics(1.0).inverse(
            PoseVelocity2dDual.constant<Time>(powers, 1),
        )

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftBack.power = wheelVels.leftBack[0] / maxPowerMag
        rightBack.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }

    private fun drawPoseHistory(c: com.acmerobotics.dashboard.canvas.Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y
            i++
        }

        c.setStrokeWidth(1)
        c.strokePolyline(xPoints, yPoints)
    }

    inner class FollowTrajectoryAction(val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps = range(
                0.0,
                timeTrajectory.path.length(),
                maxOf(2, ceil(timeTrajectory.path.length() / 2).toInt()),
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double = if (beginTs < 0) {
                beginTs = now()
                0.0
            } else {
                now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = timeTrajectory[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate() ?: PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)

            val command = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain,
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick,
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower),
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            p.put("x", localizer.getPose().position.x)
            p.put("y", localizer.getPose().position.y)
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()))

            val error = txWorldTarget.value().minusExp(localizer.getPose())
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.getPose())

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(fieldOverlay: com.acmerobotics.dashboard.canvas.Canvas) {
            fieldOverlay.setStroke("#4CAF507A")
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double = if (beginTs < 0) {
                beginTs = now()
                0.0
            } else {
                now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = turn[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate() ?: PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)

            val command = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain,
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick,
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower),
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.getPose())

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(fieldOverlay: com.acmerobotics.dashboard.canvas.Canvas) {
            fieldOverlay.setStroke("#7C4DFF7A")
            fieldOverlay.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder = TrajectoryActionBuilder(
        { turn: TimeTurn -> TurnAction(turn) },
        { t: TimeTrajectory -> FollowTrajectoryAction(t) },
        TrajectoryBuilderParams(
            1e-6,
            ProfileParams(0.25, 0.1, 1e-2),
        ),
        beginPose, 0.0,
        defaultTurnConstraints,
        defaultVelConstraint, defaultAccelConstraint,
    )

    companion object {
        @JvmStatic
        val PARAMS = Params()
    }
}





