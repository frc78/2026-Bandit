package frc.robot.lib.projectile

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d

data class ShotVector(val turretAngle: Rotation2d, val params: ShotParams)

interface ShooterController {
    fun calculate(
        shooterPosition: Translation2d,
        shooterVelocity: Translation2d,
        goalPosition: Translation2d,
        latencyCompensator: Double,
    ): ShotVector
}
