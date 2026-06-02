package frc.robot

import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Intake
import frc.robot.subsystems.SwerveDrivetrain

object OI {

    val driveController = XboxController(0)
    val manipController = XboxController(1)

    val sysIdController = XboxController(2)
    private const val POV_UP = 0
    private const val POV_RIGHT = 90
    private const val POV_DOWN = 180
    private const val POV_LEFT = 270

    // Called to update buttons that care about toggling.  If yButtonPressed were checked multiple
    // times the first would consume the press, so check once per loop and update a variable for
    // subsystems to reference.
    var lastPOV = -1

    fun periodic() {
        if (manipController.leftBumperButtonPressed) isDeployed = !isDeployed
        if (driveController.leftBumperButtonPressed) isShooting = !isShooting
        if (driveController.startButtonPressed) {
            if (Climber.currentState == Climber.ClimberState.Climbed)
                Climber.currentState = Climber.ClimberState.Unclimbed
            else if (Climber.currentState == Climber.ClimberState.Unclimbed) {
                Climber.currentState = Climber.ClimberState.Climbed
            }
        }

        if (manipController.aButtonPressed) overridePeriodTimer = !overridePeriodTimer

        val manipPov = manipController.pov
        if (manipPov == 270 && lastPOV != 270) { // formerly x button
            overrideFixedHub = !overrideFixedHub
            overrideFixedFeed = false
        }
        if (manipPov == 90 && lastPOV != 90) { // formerly b button
            overrideFixedFeed = !overrideFixedFeed
            overrideFixedHub = false
        }
        lastPOV = manipPov
    }

    // Intake Subsystem
    // True = deployed and intaking, False = retracted and still rollers
    // Don't want to deploy randomly when enabling, but want to stay deployed if coming out of auto
    var isDeployed =
        Intake.currentState == Intake.IntakeState.Intaking ||
            Intake.currentState == Intake.IntakeState.Deployed

    fun intakeRollersActive() = manipController.yButton

    fun jostleIntake() = manipController.rightBumperButton

    // Indexer & Shooter Subsystem

    // Shooter Subsystem
    var isShooting = false // True = shooting, False = not shooting

    fun isShootingHold() = driveController.leftBumperButton

    var overridePeriodTimer =
        false // True = allow shooting whenever, False = account for hub being active
    // Only up to 1 can be true at once
    var overrideFixedHub = false
    var overrideFixedFeed = false

    // Chassis Subsystem
    fun crossBump() = false // driveController.aButton

    fun xBrake() = driveController.rightBumperButton // driveController.xButton

    fun alignToTower() =
        driveController.backButton // driverLT() > 0.5 // false // driveController.bButton

    fun orientToHub() = false // driveController.leftBumperButton

    fun snapNorth() = driveController.yButton // driveController.pov == POV_UP

    fun snapEast() = driveController.bButton // driveController.pov == POV_RIGHT

    fun snapSouth() = driveController.aButton // driveController.pov == POV_DOWN

    fun snapWest() = driveController.xButton // driveController.pov == POV_LEFT

    private fun rawVelocityY() = -driveController.leftX

    private fun rawVelocityX() = -driveController.leftY

    // Slow mode from 100% to 25% via right trigger
    fun velocityX() = SwerveDrivetrain.MaxSpeed * rawVelocityX() * (1.0 - 0.75 * driverRT())

    fun velocityY() = SwerveDrivetrain.MaxSpeed * rawVelocityY() * (1.0 - 0.75 * driverRT())

    // Reduced speed for while shooting (15% of max)
    fun shootingVelocityX() = SwerveDrivetrain.MaxSpeed * rawVelocityX() * 0.15

    fun shootingVelocityY() = SwerveDrivetrain.MaxSpeed * rawVelocityY() * 0.15

    fun rawRotation() = -driveController.rightX

    // Slow mode from 100% to 55% via right trigger
    fun rotation() = SwerveDrivetrain.MaxAngularRate * rawRotation() * (1.0 - 0.45 * driverRT())

    // Reduced rotation for while shooting (30% of max)
    fun shootingRotation() = SwerveDrivetrain.MaxAngularRate * rawRotation() * 0.3

    fun driverRT() = driveController.rightTriggerAxis

    fun driverLT() = driveController.leftTriggerAxis

    fun turretSysId() = sysIdController.aButton

    fun turretSysIdPressed() = sysIdController.aButtonPressed

    fun turretSysIdReleased() = sysIdController.aButtonReleased

    fun shooterSysIdPressed() = sysIdController.bButtonPressed

    fun shooterSysIdReleased() = sysIdController.bButtonReleased

    fun turretPIDTuning() = sysIdController.xButton

    fun hoodPIDTuning() = sysIdController.yButton

    fun intakePIDTuning() = sysIdController.rightBumperButton

    fun flywheelPIDTuning() = sysIdController.leftBumperButton

    fun manualClimberUp() = sysIdController.pov == POV_UP

    fun manualClimberDown() = sysIdController.pov == POV_DOWN
}
