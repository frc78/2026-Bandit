package frc.robot.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import frc.robot.FieldLocation
import frc.robot.lib.centimeters
import frc.robot.lib.degrees
import frc.robot.lib.feet
import frc.robot.lib.inches
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Climber.ClimberState
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Indexer.IndexerState
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.SwerveDrivetrain

object LBRBOC : Auto {

    enum class Step {
        AutoStart,
        DriveToNeutralZone,
        DriveToFuel,
        Intake,
        DriveToAllianceZone,
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

    val bumpRotation = Rotation2d(45.degrees)

    override fun init() {
        step = Step.AutoStart
    }

    private val autoStartPoint by FieldLocation(3.496, 5.737, Rotation2d(45.degrees))
    private val enterFuel by FieldLocation(8.0, 6.0, Rotation2d(0.degrees))
    private val spinIntake by FieldLocation(8.0, 3.25, Rotation2d((-90).degrees))
    private val enterAllianceZone by FieldLocation(3.496, 2.463, Rotation2d((-45).degrees))
    private val enterOutpost by FieldLocation(0.665, 0.6, Rotation2d.kPi)
    private val enterTower by FieldLocation(1.092, 2.683, Rotation2d((-90).degrees))
    private val enterTowerAlignment by FieldLocation(.990, 2.877, Rotation2d((-90).degrees))

    override fun run() {
        when (step) {
            Step.AutoStart -> {
                if (RobotBase.isSimulation()) {
                    SwerveDrivetrain.resetPose(autoStartPoint)
                }
                Intake.currentState = Intake.IntakeState.Retracted
                step = Step.DriveToNeutralZone
            }
            Step.DriveToNeutralZone -> {
                if (SwerveDrivetrain.driveToPose(enterFuel, 7.feet, alwaysMaxSpeed = true)) {
                    Intake.currentState = Intake.IntakeState.Intaking
                    step = Step.DriveToFuel
                }
            }
            Step.DriveToFuel -> {
                if (SwerveDrivetrain.driveToPose(enterFuel, 2.feet, alwaysMaxSpeed = true)) {
                    step = Step.Intake
                }
            }
            Step.Intake -> {
                if (
                    SwerveDrivetrain.driveToPose(
                        spinIntake,
                        1.feet,
                        maxSpeed = 1.0,
                        alwaysMaxSpeed = true,
                    )
                ) {
                    step = Step.DriveToAllianceZone
                }
            }
            Step.DriveToAllianceZone -> {
                Intake.currentState = Intake.IntakeState.Retracted
                if (
                    SwerveDrivetrain.driveToPose(enterAllianceZone, 1.feet, alwaysMaxSpeed = true)
                ) {
                    step = Step.DriveToOutpost
                }
            }
            Step.DriveToOutpost -> {
                Shooter.flywheelState = Shooter.FlywheelState.Firing
                Indexer.currentState = IndexerState.Indexing
                Intake.currentState = Intake.IntakeState.Jostle
                if (SwerveDrivetrain.driveToPose(enterOutpost, 1.inches, 2.0)) {
                    SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
                }
            }
            Step.DriveToTower -> {
                if (SwerveDrivetrain.driveToPose(enterTower, 1.centimeters, 1.0)) {
                    step = Step.AlignToClimb
                }
            }
            Step.AlignToClimb -> {
                if (SwerveDrivetrain.driveToPose(enterTowerAlignment, 1.centimeters, 0.5)) {
                    step = Step.Climb
                }
            }
            Step.Climb -> {
                Climber.currentState = ClimberState.Climbed
                SwerveDrivetrain.driveToPose(enterTowerAlignment, 1.centimeters, 0.5)
            }
            Step.Finished -> {
                SwerveDrivetrain.setControl(SwerveRequest.FieldCentric())
            }
        }
    }
}
