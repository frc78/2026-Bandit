package frc.robot.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import frc.robot.FieldLocation
import frc.robot.lib.degrees
import frc.robot.lib.feet
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Indexer.IndexerState
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.SwerveDrivetrain

object LBLB : Auto {

    enum class Step {
        AutoStart,
        DriveToFuel,
        Intake,
        DriveToAllianceZone,
        DriveToMidPoint,
        Shoot,
        Finished;

        operator fun inc(): Step {
            return entries.getOrNull(ordinal + 1) ?: this
        }
    }

    var step = Step.AutoStart

    override fun init() {
        step = Step.AutoStart
    }

    val autoStartPoint by FieldLocation(3.496, 5.5, Rotation2d((-45).degrees))
    val neutralZone by FieldLocation(6.0, 6.0, Rotation2d((-45).degrees))
    val enterFuel by FieldLocation(7.70, 7.0, Rotation2d((-80).degrees))
    val spinIntake by FieldLocation(7.70, 4.75, Rotation2d((-80).degrees))
    val enterAllianceZone by FieldLocation(1.67, 6.0, Rotation2d(0.degrees))

    override fun run() {
        when (step) {
            Step.AutoStart -> {
                if (RobotBase.isSimulation()) {
                    SwerveDrivetrain.resetPose(autoStartPoint)
                }
                step = Step.DriveToMidPoint
            }
            Step.DriveToMidPoint -> {
                if (SwerveDrivetrain.driveToPose(neutralZone, 1.feet, alwaysMaxSpeed = true)) {
                    step = Step.DriveToFuel
                    Intake.currentState = Intake.IntakeState.Intaking
                }
            }
            Step.DriveToFuel -> {
                Intake.currentState = Intake.IntakeState.Intaking
                if (SwerveDrivetrain.driveToPose(enterFuel, 1.feet, minSpeed = 1.25)) {
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
                    SwerveDrivetrain.driveToPose(enterAllianceZone, 1.feet, alwaysMaxSpeed = false)
                ) {
                    step = Step.Shoot
                }
            }
            Step.Shoot -> {
                SwerveDrivetrain.driveToPose(enterAllianceZone)
                Indexer.currentState = IndexerState.Indexing
                Intake.currentState = Intake.IntakeState.Jostle
                Shooter.flywheelState = Shooter.FlywheelState.Firing
            }
            Step.Finished -> {
                SwerveDrivetrain.setControl(SwerveRequest.FieldCentric())
            }
        }
    }
}
