package frc.robot.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import frc.robot.FieldConstants
import frc.robot.FieldLocation
import frc.robot.lib.degrees
import frc.robot.lib.feet
import frc.robot.lib.meters
import frc.robot.lib.seconds
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.SwerveDrivetrain

object OD : Auto {
    enum class Step {
        AutoStart,
        DriveToOutpost,
        driveToMiddleLower,
        DriveToMiddleUpper,
        driveToDepo,
        driveToWall,
        Finished;

        operator fun inc(): Step {
            return entries.getOrNull(ordinal + 1) ?: this
        }
    }

    var step = Step.AutoStart

    private val autoStartPoint by FieldLocation(3.552, 4.017, Rotation2d((180).degrees))
    private val outpost by FieldLocation(0.665, .674, Rotation2d(180.degrees))
    private val lowerMiddle by
        FieldLocation(
            2.0.meters,
            (FieldConstants.MIDPOINT.y.meters - 2.0.meters),
            Rotation2d(90.degrees),
        )
    private val upperMiddle by
        FieldLocation(
            2.0.meters,
            (FieldConstants.MIDPOINT.y.meters + 1.meters),
            Rotation2d(90.degrees),
        )
    private val outpostTimer = Timer()
    private val driveToWall by FieldLocation(0.5, 7.5, Rotation2d(90.degrees))
    private val depot by FieldLocation(0.45, 4.776, Rotation2d(90.degrees))

    override fun init() {
        step = Step.AutoStart
    }

    override fun run() {
        when (step) {
            Step.AutoStart -> {
                SwerveDrivetrain.resetPose(autoStartPoint)
                Intake.currentState = Intake.IntakeState.Deployed
                step++
            }
            Step.DriveToOutpost -> {
                Shooter.flywheelState = Shooter.FlywheelState.Firing
                SwerveDrivetrain.driveToPose(outpost, maxSpeed = 2.0)
                if (SwerveDrivetrain.driveToPose(outpost, 0.1.feet)) {
                    SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
                    outpostTimer.restart()
                    step++
                }
            }
            Step.driveToMiddleLower -> {
                if (outpostTimer.hasElapsed(3.seconds)) {
                    if (
                        SwerveDrivetrain.driveToPose(lowerMiddle, 0.25.feet, alwaysMaxSpeed = true)
                    ) {
                        Intake.currentState = Intake.IntakeState.Intaking
                        step++
                    }
                }
            }
            Step.DriveToMiddleUpper -> {
                if (SwerveDrivetrain.driveToPose(upperMiddle, 0.25.feet, alwaysMaxSpeed = true)) {
                    step++
                }
            }
            Step.driveToDepo -> {
                if (SwerveDrivetrain.driveToPose(depot, 0.5.feet, 1.0)) {
                    step++
                }
            }
            Step.driveToWall -> {
                if (SwerveDrivetrain.driveToPose(driveToWall, 0.25.feet, 1.0)) {
                    step++
                }
            }
            Step.Finished -> {
                SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())
            }
        }
    }
}
