package frc.robot.auto

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.lib.inches
import frc.robot.subsystems.SwerveDrivetrain

class SampleAuto {
    fun run() {
        SwerveDrivetrain.driveToPose(Pose2d(325.61.inches, 158.84.inches, Rotation2d.kCCW_90deg))
    }
}
