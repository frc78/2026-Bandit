package frc.robot.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import frc.robot.FieldLocation
import frc.robot.lib.degrees
import frc.robot.lib.feet
import frc.robot.lib.seconds
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Intake.IntakeState
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.SwerveDrivetrain
import org.littletonrobotics.junction.Logger

object RBRBOC : Auto {

    enum class Step {
        AutoStart,
        CrossBump,
        DriveToFuelStart,
        DriveThroughFuel,
        DriveToMidNeutral,
        DriveToAllianceZone,
        DriveToOutpost,
        WaitThenJostle,
        DriveToTower,
        AlignToClimb,
        Climb,
        Finished;

        operator fun inc(): Step {
            return entries.getOrNull(ordinal + 1) ?: this
        }
    }

    var step = Step.AutoStart

    private val autoStartPoint by FieldLocation(3.36, 2.233, Rotation2d(0.degrees))
    private val neutralZone by FieldLocation(5.0, 2.233, Rotation2d(0.degrees))
    private val rightOfFuel by FieldLocation(7.6, 1.0, Rotation2d(90.degrees))
    private val fuel by FieldLocation(7.6, 4.0, Rotation2d(90.degrees))
    private val outpost by FieldLocation(0.665, .674, Rotation2d(180.degrees))
    private val outpostTimer = Timer()
    private val tower by FieldLocation(1.05, 2.6, Rotation2d((-90).degrees))
    private val midNeutral by FieldLocation(5.897, 2.556, Rotation2d(45.degrees))

    override fun init() {
        step = Step.AutoStart
    }

    override fun run() {
        Logger.recordOutput("auto/step", step.name)
        when (step) {
            Step.AutoStart -> {
                Shooter.flywheelState = Shooter.FlywheelState.Still
                Indexer.currentState = Indexer.IndexerState.Still
                SwerveDrivetrain.resetPose(autoStartPoint)
                Intake.currentState = IntakeState.Retracted
                step++
            }
            Step.CrossBump -> {
                if (SwerveDrivetrain.driveToPose(neutralZone, 1.feet, alwaysMaxSpeed = true)) {
                    Intake.currentState = IntakeState.Deployed
                    step++
                }
            }
            Step.DriveToFuelStart -> {
                if (SwerveDrivetrain.driveToPose(rightOfFuel, 1.feet, alwaysMaxSpeed = true)) {
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
                    SwerveDrivetrain.driveToPose(neutralZone)
                    Intake.currentState = IntakeState.Deployed
                    step++
                }
            }
            Step.DriveToMidNeutral -> {
                SwerveDrivetrain.driveToPose(midNeutral, 0.5.feet, alwaysMaxSpeed = true)
                if (SwerveDrivetrain.driveToPose(midNeutral, 0.5.feet)) {
                    step++
                }
            }

            Step.DriveToAllianceZone -> {
                if (SwerveDrivetrain.driveToPose(autoStartPoint, 2.feet, alwaysMaxSpeed = true)) {
                    step++
                }
            }
            Step.DriveToOutpost -> {
                Shooter.flywheelState = Shooter.FlywheelState.Firing
                if (SwerveDrivetrain.driveToPose(outpost, 1.0.feet, maxSpeed = 1.0)) {
                    outpostTimer.restart()
                    step++
                }
            }
            Step.WaitThenJostle -> {
                if (SwerveDrivetrain.driveToPose(outpost, .25.feet)) {
                    SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
                }
                if (outpostTimer.hasElapsed(5.0)) {
                    Intake.currentState = IntakeState.Jostle
                }
            }
            Step.DriveToTower -> {
                if (outpostTimer.hasElapsed(3.seconds)) {
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
