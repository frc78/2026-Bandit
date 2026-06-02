package frc.robot.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import frc.robot.FieldLocation
import frc.robot.lib.degrees
import frc.robot.lib.feet
import frc.robot.lib.seconds
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.SwerveDrivetrain

object DC : Auto {
    enum class Step {
        AutoStart,
        DriveToDepot,
        DriveToTower,
        AlignToClimb,
        Climb,
        Finished;

        operator fun inc(): Step {
            return entries.getOrNull(ordinal + 1) ?: this
        }
    }

    var step = Step.AutoStart

    private val autoStartPoint by FieldLocation(3.36, 5.737, Rotation2d(45.degrees))
    private val depot by FieldLocation(0.671, 5.976, Rotation2d(180.degrees))
    private val depotTimer = Timer()
    private val tower by FieldLocation(1.05, 4.6, Rotation2d((90).degrees))

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
            Step.DriveToDepot -> {
                Shooter.flywheelState = Shooter.FlywheelState.Firing
                Intake.currentState = Intake.IntakeState.Intaking
                SwerveDrivetrain.driveToPose(depot, maxSpeed = 1.0)
                if (SwerveDrivetrain.driveToPose(depot, 0.1.feet)) {
                    depotTimer.restart()
                    SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
                    Intake.currentState = Intake.IntakeState.Jostle
                }
            }

            Step.DriveToTower -> {
                if (depotTimer.hasElapsed(3.seconds)) {
                    SwerveDrivetrain.driveToPose(tower)
                    if (SwerveDrivetrain.driveToPose(tower, 0.1.feet)) {}
                }
            }

            Step.AlignToClimb -> {
                step++
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
