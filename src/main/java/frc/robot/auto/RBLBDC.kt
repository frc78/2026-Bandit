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
import kotlin.math.PI

object RBLBDC : Auto {

    enum class Step {
        AutoStart,
        DriveToNeutralZone,
        DriveToFuel,
        Intake,
        DriveToAllianceZone,
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

    val bumpRotation = Rotation2d(45.degrees)

    override fun init() {
        step = Step.AutoStart
    }

    private val autoStartPoint by FieldLocation(3.41, 2.463, Rotation2d(45.degrees))
    private val enterFuel by FieldLocation(7.5, 2.163, Rotation2d(0.degrees))
    private val spinIntake by FieldLocation(7.5, 6.201, Rotation2d.kCCW_90deg)
    private val enterAllianceZone by FieldLocation(3.496, 5.5, Rotation2d(45.degrees))
    private val enterDepot by FieldLocation(0.671, 5.976, Rotation2d.kPi)
    private val enterTower by FieldLocation(1.092, 4.636, Rotation2d.kCCW_90deg)
    private val enterTowerAlignment by FieldLocation(.990, 4.6360, Rotation2d.kCCW_90deg)

    override fun run() {
        when (step) {
            Step.AutoStart -> {
                if (RobotBase.isSimulation()) {
                    SwerveDrivetrain.resetPose(autoStartPoint)
                }
                step = Step.DriveToNeutralZone
            }
            Step.DriveToNeutralZone -> {
                if (SwerveDrivetrain.driveToPose(enterFuel, 7.feet, alwaysMaxSpeed = true)) {
                    Intake.currentState = Intake.IntakeState.Intaking
                    step = Step.DriveToFuel
                }
            }
            Step.DriveToFuel -> {
                if (SwerveDrivetrain.driveToPose(enterFuel, 1.feet, alwaysMaxSpeed = true)) {
                    step = Step.Intake
                }
            }
            Step.Intake -> {
                if (
                    SwerveDrivetrain.driveToPose(
                        spinIntake,
                        1.feet,
                        maxSpeed = 1.25,
                        alwaysMaxSpeed = true,
                    )
                ) {
                    step = Step.DriveToAllianceZone
                }
            }
            Step.DriveToAllianceZone -> {
                if (
                    SwerveDrivetrain.driveToPose(enterAllianceZone, 1.feet, alwaysMaxSpeed = true)
                ) {
                    step = Step.DriveToDepot
                }
            }
            Step.DriveToDepot -> {
                Shooter.flywheelState = Shooter.FlywheelState.Firing
                Intake.currentState = Intake.IntakeState.Jostle
                Indexer.currentState = IndexerState.Indexing
                if (SwerveDrivetrain.driveToPose(enterDepot, 1.inches, 1.0, PI / 2)) {
                    SwerveDrivetrain.setControl(SwerveRequest.FieldCentric())
                }
            }
            Step.DriveToTower -> {
                if (SwerveDrivetrain.driveToPose(enterTower, 1.centimeters, 0.5)) {
                    step = Step.AlignToClimb
                }
            }
            Step.AlignToClimb -> {
                if (SwerveDrivetrain.driveToPose(enterTowerAlignment, 1.centimeters, .25)) {
                    step = Step.Climb
                }
            }
            Step.Climb -> {
                Climber.currentState = ClimberState.Climbed
                SwerveDrivetrain.driveToPose(enterTowerAlignment, 1.centimeters, .25)
            }
            Step.Finished -> {
                SwerveDrivetrain.setControl(SwerveRequest.FieldCentric())
            }
        }
    }
}
