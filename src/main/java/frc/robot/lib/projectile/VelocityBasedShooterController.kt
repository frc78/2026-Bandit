package frc.robot.lib.projectile

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.lib.seconds

class VelocityBasedShooterController(private val lut: ShooterLUT) : ShooterController {

    override fun calculate(
        shooterPosition: Translation2d,
        shooterVelocity: Translation2d,
        goalPosition: Translation2d,
        latencyCompensator: Double,
    ): ShotVector {
        // Account for latency by predicting the robots position a short time in the future
        val futurePos: Translation2d = shooterPosition + shooterVelocity * latencyCompensator

        val toGoal: Translation2d = goalPosition - futurePos
        val distance: Double = toGoal.norm
        // Unit vector reprenting the target direction
        val targetDirection: Translation2d = toGoal / distance

        val baseline: ShotParams = lut.getParamsForDistance(distance)
        val baselineVelocity: Double = distance / baseline.tof.seconds

        val targetVelocity: Translation2d = targetDirection * baselineVelocity

        val shotVelocity: Translation2d = targetVelocity - shooterVelocity

        val turretAngle: Rotation2d = shotVelocity.angle
        val requiredVelocity: Double = shotVelocity.norm

        return ShotVector(turretAngle, lut.getParamsForVelocity(requiredVelocity))
    }
}
