 package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

/** Flight Recorder messages for Road Runner 1.0 (ported from the official quickstart). */

class PoseMessage(pose: Pose2d) {
    val timestamp = System.nanoTime()
    val x = pose.position.x
    val y = pose.position.y
    val heading = pose.heading.toDouble()
}

class DriveCommandMessage(poseVelocity: PoseVelocity2dDual<Time>) {
    val timestamp = System.nanoTime()
    val forwardVelocity = poseVelocity.linearVel.x[0]
    val forwardAcceleration = poseVelocity.linearVel.x[1]
    val lateralVelocity = poseVelocity.linearVel.y[0]
    val lateralAcceleration = poseVelocity.linearVel.y[1]
    val angularVelocity = poseVelocity.angVel[0]
    val angularAcceleration = poseVelocity.angVel[1]
}

class MecanumCommandMessage(
    voltage: Double,
    val leftFrontPower: Double,
    val leftBackPower: Double,
    val rightBackPower: Double,
    val rightFrontPower: Double,
) {
    val timestamp = System.nanoTime()
    val voltage = voltage
}

class MecanumLocalizerInputsMessage(
    val leftFront: PositionVelocityPair,
    val leftBack: PositionVelocityPair,
    val rightBack: PositionVelocityPair,
    val rightFront: PositionVelocityPair,
    angles: YawPitchRollAngles,
) {
    val timestamp = System.nanoTime()
    val yaw = angles.getYaw(AngleUnit.RADIANS)
    val pitch = angles.getPitch(AngleUnit.RADIANS)
    val roll = angles.getRoll(AngleUnit.RADIANS)
}
