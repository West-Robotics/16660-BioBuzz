package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d

/**
 * Interface for localization methods, matching the Road Runner 1.0 quickstart
 * (`com.acmerobotics.roadrunner` core has no Localizer abstraction, so the drive
 * class owns localization; this interface keeps the localization implementation
 * swappable, e.g. dead-wheels or Pinpoint could replace the built-in mecanum
 * localizer later).
 */
interface Localizer {
    fun setPose(pose: Pose2d)

    /**
     * Returns the current pose estimate.
     * NOTE: Does not update the pose estimate;
     * you must call update() to update the pose estimate.
     */
    fun getPose(): Pose2d

    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    fun update(): PoseVelocity2d?
}

/**
 * Hook that allows an external estimator (the hybrid visual localization system)
 * to correct the Road Runner pose estimate.
 *
 * This is the *supported integration point* between the vision system and Road
 * Runner: [MecanumDrive.updatePoseEstimate] calls [correct] after the normal
 * mecanum/IMU localizer update, and writes the returned pose back into the
 * localizer. Implementations MUST:
 *  - be fast (called every loop iteration),
 *  - be safe (return the input pose unchanged when vision is unavailable or
 *    not confident),
 *  - never teleport the pose (corrections must be clamped).
 *
 * The hybrid visual estimator in `org.firstinspires.ftc.teamcode.vision`
 * implements this interface; see [org.firstinspires.ftc.teamcode.vision.HybridPoseEstimator].
 */
interface PoseCorrector {
    /**
     * @param rrPose the pose Road Runner currently believes the robot is at
     * @param rrVelocity the current Road Runner velocity estimate (may be null)
     * @param timestampNanos current time, used to reject stale vision data
     * @return the corrected pose (or the input pose if no correction applies)
     */
    fun correct(rrPose: Pose2d, rrVelocity: PoseVelocity2d?, timestampNanos: Long): Pose2d
}
