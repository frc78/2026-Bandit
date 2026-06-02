package frc.robot.vision

import com.ctre.phoenix6.Utils
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.SwerveDrivetrain
import kotlin.jvm.optionals.getOrNull
import kotlin.math.pow
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

class Camera(val name: String, val transform: Transform3d) {
    val cam = PhotonCamera(name)
    val robotToCamera2d =
        Transform2d(transform.translation.toTranslation2d(), transform.rotation.toRotation2d())

    val estimator =
        PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
            transform,
        )

    private val singleTagStds: Matrix<N3, N1> = VecBuilder.fill(0.01, 0.01, 1.0)
    private val multiTagStds: Matrix<N3, N1> = VecBuilder.fill(0.002, 0.002, 0.1)
    private val outOfRangeStds =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)

    fun updatePoseEstimate() {
        estimator.addHeadingData(
            SwerveDrivetrain.state.Timestamp -
                (Utils.getCurrentTimeSeconds() - Timer.getFPGATimestamp()),
            SwerveDrivetrain.state.Pose.rotation,
        )
        cam.allUnreadResults.forEach { result ->
            estimator.estimateCoprocMultiTagPose(result).getOrNull()?.let {
                val stdDevs = getStdDevsForEstimate(it)
                SwerveDrivetrain.addVisionMeasurement(
                    it.estimatedPose.toPose2d(),
                    it.timestampSeconds,
                    stdDevs,
                )
            }
        }
    }

    private fun getStdDevsForEstimate(estimate: EstimatedRobotPose): Matrix<N3, N1> {
        val validTargets =
            estimate.targetsUsed.mapNotNull {
                estimator.fieldTags.getTagPose(it.fiducialId).getOrNull()
            }
        if (validTargets.isEmpty()) {
            return outOfRangeStds
        }
        val minDist =
            validTargets.minOf { it.translation.getDistance(estimate.estimatedPose.translation) }
        if (validTargets.size == 1 && minDist > 2) {
            return outOfRangeStds
        }
        val currentStds = if (validTargets.size > 1) multiTagStds else singleTagStds

        // We need to improve standard deviation calculations
        return currentStds * (1 + (minDist.pow(2) / 100)) // was / 15
    }
}
