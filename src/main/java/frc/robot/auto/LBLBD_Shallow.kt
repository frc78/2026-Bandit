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
import frc.robot.subsystems.Intake.IntakeState
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.SwerveDrivetrain
import org.littletonrobotics.junction.Logger

object LBLBD_Shallow : Auto {
    enum class Step {
        AutoStart,
        CrossBump,
        DriveToFuelStart,
        DriveThroughFuel,
        BackToBump,
        DriveToAllianceZone,
        DriveToDepot,
        DriveToWall,
        DriveToTower,
        AlignToClimb,
        Climb,
        Finished;

        operator fun inc(): Step {
            return entries.getOrNull(ordinal + 1) ?: this
        }
    }

    var step = Step.AutoStart

    private val autoStartPoint by FieldLocation(3.36, 5.737, Rotation2d((-45).degrees))
    private val neutralZone by FieldLocation(5.0, 5.6, Rotation2d((-45).degrees))
    private val leftOfFuel by FieldLocation(7.5, 7.0, Rotation2d((-90).degrees))
    private val fuel by FieldLocation(7.5, 4.8, Rotation2d((-90).degrees))
    private val depot by FieldLocation(0.45, 4.776, Rotation2d(90.degrees))
    private val depotTimer = Timer()
    private val allianceWall by FieldLocation(2.8, 5.7, Rotation2d(90.degrees))
    private val tower by FieldLocation(1.05, 4.6, Rotation2d(90.degrees))
    private val wall by FieldLocation(0.5, 7.15, Rotation2d(90.degrees))

    override fun init() {
        step = Step.AutoStart
    }

    override fun run() {
        Logger.recordOutput("auto/step", step.name)
        when (step) {
            Step.AutoStart -> {
                if (RobotBase.isSimulation()) {
                    SwerveDrivetrain.resetPose(autoStartPoint)
                }
                Intake.currentState = IntakeState.Retracted
                step++
            }
            Step.CrossBump -> {
                if (SwerveDrivetrain.driveToPose(neutralZone, 2.feet, alwaysMaxSpeed = true)) {
                    Intake.currentState = IntakeState.Intaking
                    step++
                }
            }
            Step.DriveToFuelStart -> {
                if (SwerveDrivetrain.driveToPose(leftOfFuel, 2.feet, alwaysMaxSpeed = true)) {
                    Intake.currentState = IntakeState.Intaking
                    step++
                }
            }

            Step.DriveThroughFuel -> {
                if (
                    SwerveDrivetrain.driveToPose(
                        fuel,
                        0.5.feet,
                        alwaysMaxSpeed = true,
                        maxSpeed = 1.0,
                    )
                ) {
                    Intake.currentState = IntakeState.Deployed
                    step++
                }
            }

            Step.BackToBump -> {
                if (SwerveDrivetrain.driveToPose(neutralZone, 1.feet, alwaysMaxSpeed = true)) {
                    step++
                }
            }

            Step.DriveToAllianceZone -> {
                if (SwerveDrivetrain.driveToPose(allianceWall, 2.feet, alwaysMaxSpeed = true)) {
                    step++
                }
            }

            Step.DriveToDepot -> {
                //                Intake.currentState = IntakeState.Jostle
                Shooter.flywheelState = Shooter.FlywheelState.Firing
                SwerveDrivetrain.driveToPose(depot, maxSpeed = 1.0, maxRotation = 1.0)
                if (SwerveDrivetrain.driveToPose(depot, 0.5.feet)) {
                    depotTimer.restart()
                    Intake.currentState = IntakeState.Intaking
                    step++
                }
            }

            Step.DriveToWall -> {
                SwerveDrivetrain.driveToPose(wall, maxSpeed = 1.0)
                if (SwerveDrivetrain.driveToPose(wall, 0.1.feet)) {
                    Intake.currentState = IntakeState.Jostle
                    SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
                }
            }

            Step.DriveToTower -> {
                if (depotTimer.hasElapsed(3.seconds)) {
                    SwerveDrivetrain.driveToPose(tower)
                    if (SwerveDrivetrain.driveToPose(tower, 0.1.feet))
                        SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
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
