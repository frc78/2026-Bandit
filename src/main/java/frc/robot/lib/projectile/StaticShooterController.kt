package frc.robot.lib.projectile

import edu.wpi.first.math.geometry.Translation2d

class StaticShooterController(val lut: ShooterLUT) : ShooterController {
    override fun calculate(
        shooterPosition: Translation2d,
        shooterVelocity: Translation2d,
        goalPosition: Translation2d,
        latencyCompensator: Double,
    ): ShotVector {

        return ShotVector(
            (goalPosition - shooterPosition).angle,
            lut.getParamsForDistance((goalPosition - shooterPosition).norm),
        )
    }
}
