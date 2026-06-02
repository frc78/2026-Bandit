package frc.robot.lib.projectile

import edu.wpi.first.math.geometry.Translation2d
import frc.robot.lib.meters
import frc.robot.lib.seconds
import kotlin.math.abs
import org.littletonrobotics.junction.Logger

class IterativeBasedShooterController(private val lut: ShooterLUT) : ShooterController {

    override fun calculate(
        shooterPosition: Translation2d,
        shooterVelocity: Translation2d,
        goalPosition: Translation2d,
        latencyCompensator: Double,
    ): ShotVector {

        val dist: Double = (goalPosition - shooterPosition).norm
        Logger.recordOutput("base target dist", dist.meters)
        Logger.recordOutput("base target angle", (goalPosition - shooterPosition).angle)

        var iterativeShot =
            ShotVector((goalPosition - shooterPosition).angle, lut.getParamsForDistance(dist))

        var adjustedVector = (goalPosition - shooterPosition)
        for (i in 1..5) {
            val shotTime = iterativeShot.params.tof.seconds
            val adjustedTarget = goalPosition - (shooterVelocity * shotTime)
            adjustedVector = adjustedTarget - shooterPosition

            val newDist = adjustedVector.norm
            val newShot = lut.getParamsForDistance(newDist)

            val newShotTime = newShot.tof.seconds
            iterativeShot = ShotVector(adjustedVector.angle, newShot)
            if (abs(shotTime - newShotTime) < 0.010) {
                break
            }
        }
        Logger.recordOutput(
            "shooter/adjustedTarget",
            shooterPosition,
            shooterPosition + adjustedVector,
        )
        return iterativeShot
    }
}
