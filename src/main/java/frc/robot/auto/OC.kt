package frc.robot.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import frc.robot.FieldLocation
import frc.robot.lib.centimeters
import frc.robot.lib.degrees
import frc.robot.lib.feet
import frc.robot.lib.seconds
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.SwerveDrivetrain

object OC : Auto {
    enum class Step {
        AutoStart,
        DriveToOutpost,
        DriveToTower,
        AlignToClimb,
        Climb,
        Finished;

        operator fun inc(): Step {
            return entries.getOrNull(ordinal + 1) ?: this
        }
    }

    var step = Step.AutoStart

    private val autoStartPoint by FieldLocation(3.36, 2.233, Rotation2d(45.degrees))
    private val outpost by FieldLocation(0.665, .674, Rotation2d(180.degrees))
    private val outpostTimer = Timer()
    private val tower by FieldLocation(1.05, 2.6, Rotation2d((-90).degrees))
    private val enterTowerAlignment by FieldLocation(.990, 2.877, Rotation2d((-90).degrees))
    private val moveOutFromWall by FieldLocation(0.75, .674, Rotation2d(180.degrees))

    override fun init() {
        step = Step.AutoStart
    }

    override fun run() {
        when (step) {
            Step.AutoStart -> {
                if (RobotBase.isSimulation()) {
                    SwerveDrivetrain.resetPose(autoStartPoint)
                }
                Intake.currentState = Intake.IntakeState.Retracted
                step++
            }
            Step.DriveToOutpost -> {
                Shooter.flywheelState = Shooter.FlywheelState.Firing
                SwerveDrivetrain.driveToPose(outpost, maxSpeed = 1.0)
                if (SwerveDrivetrain.driveToPose(outpost, 0.1.feet)) {
                    outpostTimer.restart()
                    step++
                }
            }
            Step.DriveToTower -> {
                if (outpostTimer.hasElapsed(3.seconds)) {
                    SwerveDrivetrain.driveToPose(moveOutFromWall)
                    if (SwerveDrivetrain.driveToPose(moveOutFromWall, 0.25.feet)) {
                        SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
                        Intake.currentState = Intake.IntakeState.Jostle
                    }
                }
            }
            Step.AlignToClimb -> {
                if (SwerveDrivetrain.driveToPose(enterTowerAlignment, 1.centimeters, 0.5)) {
                    step++
                }
            }
            Step.Climb -> {
                Climber.currentState = Climber.ClimberState.Climbed
                step++
            }
            Step.Finished -> {
                SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
            }
        }
    }
}
