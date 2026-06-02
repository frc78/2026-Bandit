package frc.robot.subsystems

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.*
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import frc.robot.FieldConstants.BLUE_HUB_CORNER_1
import frc.robot.FieldConstants.BLUE_HUB_CORNER_2
import frc.robot.FieldConstants.FEED_LEFT_POSE
import frc.robot.FieldConstants.FEED_RIGHT_POSE
import frc.robot.FieldConstants.HUB_POSE
import frc.robot.FieldConstants.RED_HUB_CORNER_1
import frc.robot.FieldConstants.RED_HUB_CORNER_2
import frc.robot.OI
import frc.robot.generated.TunerConstants
import frc.robot.lib.*
import frc.robot.lib.PeriodTimer.isHubActive
import frc.robot.lib.PeriodTimer.isNextPeriodActive
import frc.robot.lib.PeriodTimer.timeElapsedInPeriod
import frc.robot.lib.PeriodTimer.timeRemainingInPeriod
import frc.robot.lib.projectile.HubLut
import frc.robot.lib.projectile.IterativeBasedShooterController
import frc.robot.lib.projectile.ShotVector
import frc.robot.lib.radians
import frc.robot.lib.rotations
import frc.robot.subsystems.SwerveDrivetrain.robotInOpponentAllianceZone
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

object Shooter {

    object Constants {
        // 4:1 planetary stack, 10 tooth pinion, 90 tooth turret ring
        const val TURRET_GEAR_RATIO = 5.0 * 90.0 / 10.0 // 36:1
        const val HOOD_GEAR_RATIO = (44.0 / 16.0) * (36.0 / 18.0) * (182.0 / 10.0) // 100.1 : 1
        const val FLYWHEEL_GEAR_RATIO = 2.0

        val SHOOTER_OFFSET = Translation2d(-.127, .152)

        // Thresholds for determining when systems are at their target
        val TURRET_ANGLE_THRESHOLD = 4.0.degrees.rotations
        val HOOD_ANGLE_THRESHOLD = 1.5.degrees.rotations
        val FLYWHEEL_VELOCITY_THRESHOLD = 50.rpm.rotationsPerSecond

        // Fixed Shot Parameters
        val FIXED_HUB_SHOT_TURRET = 0.0.degrees
        val FIXED_HUB_SHOT_HOOD = 1.3.degrees
        val FIXED_HUB_SHOT_FLYWHEEL = 1200.0.rpm

        val FIXED_FEED_SHOT_TURRET = 0.0.degrees
        val FIXED_FEED_SHOT_HOOD = 0.0.degrees
        val FIXED_FEED_SHOT_FLYWHEEL = 1000.0.rpm
    }

    private val hubShotController = IterativeBasedShooterController(HubLut)
    private val feedShotController = IterativeBasedShooterController(HubLut)

    /**
     * Debouncer for flywheel to allow for expected RPM drops while shooting without
     * flywheelAtVelocity getting set to false and stopping the feeder
     */
    val flywheelVelocityDebouncer = Debouncer(0.2, Debouncer.DebounceType.kFalling)

    val tuningHoodAngle = LoggedNetworkNumber("shooter/tuningHoodAngleDegrees", 0.0)
    val tuningFlywheelSpeed = LoggedNetworkNumber("shooter/tuningFlywheelSpeedRPM", 0.0)

    /** timer for tracking feeder clear */
    private val clearTimer = Timer()

    // -----------------------------------------------------
    //                  Motor Setup
    // -----------------------------------------------------
    private val turretMotor =
        TalonFX(12, TunerConstants.kCANBus).apply {
            val config =
                TalonFXConfiguration().apply {
                    MotorOutput.apply { Inverted = InvertedValue.Clockwise_Positive }
                    Slot0.apply {
                        kP = 150.0
                        kV = 7.8416
                        kA = 0.52511
                        kD = 5.0
                        kS = .28679
                    }
                    Feedback.apply { SensorToMechanismRatio = Constants.TURRET_GEAR_RATIO }
                }
            configurator.apply(config)
        }

    private val hoodMotor =
        TalonFX(11, TunerConstants.kCANBus).apply {
            val config =
                TalonFXConfiguration().apply {
                    Slot0.apply {
                        kP = 800.0
                        kD = 4.0
                    }
                    SoftwareLimitSwitch.apply {
                        ForwardSoftLimitEnable = true
                        ForwardSoftLimitThreshold = 30.degrees.rotations
                        ReverseSoftLimitEnable = true
                        ReverseSoftLimitThreshold = 0.degrees.rotations
                    }
                    Feedback.FeedbackRotorOffset = -0.026
                    Feedback.apply { SensorToMechanismRatio = Constants.HOOD_GEAR_RATIO }
                    CurrentLimits.apply {
                        StatorCurrentLimit = 40.0
                        SupplyCurrentLimit = 10.0
                    }
                }
            configurator.apply(config)
        }

    private val flywheelMotor =
        TalonFX(10, TunerConstants.kCANBus).apply {
            val config =
                TalonFXConfiguration().apply {
                    Slot0.apply {
                        kP = 999999999.0
                        kS = 8.0
                    }
                    TorqueCurrent.PeakReverseTorqueCurrent = 0.0
                    Feedback.apply { SensorToMechanismRatio = Constants.FLYWHEEL_GEAR_RATIO }
                    MotorOutput.apply { Inverted = InvertedValue.Clockwise_Positive }
                    CurrentLimits.apply { StatorCurrentLimit = 80.0 }
                }
            configurator.apply(config)
        }

    private val hubCanRange = CANrange(13, TunerConstants.kCANBus)

    private val turretControlRequest = PositionVoltage(0.degrees)
    private val hoodControlRequest = PositionVoltage(0.degrees)
    private val flywheelControlRequest = VelocityTorqueCurrentFOC(0.0.rpm)

    // -----------------------------------------------------
    //                  State Setup
    // -----------------------------------------------------
    enum class TurretState { // also used for Hood, except for SysId states
        SysIdIdle,
        Disabled,
        Aiming,
        FixedFeed,
        FixedHub,
        PIDTuning,
        ShotTuning,
        SysIdQuasistaticForward,
        SysIdQuasistaticReverse,
        SysIdDynamicForward,
        SysIdDynamicReverse,
    }

    enum class FlywheelState {
        SysIdIdle,
        Firing,
        Clearing,
        PIDTuning,
        ShotTuning,
        Still,
        SysIdQuasistaticForward,
        SysIdQuasistaticReverse,
        SysIdDynamicForward,
        SysIdDynamicReverse,
    }

    var turretState = TurretState.Aiming
    var flywheelState = FlywheelState.Still

    // -----------------------------------------------------
    //                  SysId Setup
    // -----------------------------------------------------
    private val idleRequest = CoastOut()
    private val sysIdTimer = Timer()
    private val turretSysIdLog = SysIdRoutineLog("turret")

    // Created flywheel log variable
    private val flywheelSysIdLog = SysIdRoutineLog("flywheel")
    private val turretCalculator = TurretCalculator((-198).degrees, 318.degrees)

    // -----------------------------------------------------
    //              State Machine Implementations
    // -----------------------------------------------------
    fun stateActions() {
        Logger.recordOutput("shooter/turretState", turretState)
        Logger.recordOutput("shooter/flywheelState", flywheelState)
        Logger.recordOutput(
            "shooter/turret_to_hub_dist",
            getFieldRelativeShooterTranslation().minus(HUB_POSE.translation).norm,
        )
        shotVector = calculateShotVector()
        hasValidShot = hasValidShot()

        Logger.recordOutput("shooterMechanisms/flywheelAtSpeed", flywheelAtSpeed)

        // Turret (and Hood) per-state actions
        when (turretState) {
            TurretState.Disabled -> {
                // fixed turret, aiming hood
                turretMotor.setControl(turretControlRequest.withPosition(0.0))
                val hoodAngle = shotVector.params.hoodAngle
                hoodMotor.setControl(hoodControlRequest.withPosition(hoodAngle))
            }
            TurretState.ShotTuning -> {
                val targetAngle =
                    turretCalculator.calculateNewHeadingDegrees(
                        turretControlRequest.Position.rotations,
                        (shotVector.turretAngle - SwerveDrivetrain.state.Pose.rotation).measure,
                    )

                turretMotor.setControl(turretControlRequest.withPosition(targetAngle))

                Logger.recordOutput("shooter/shotTuningHoodAngleDeg", hoodAngle.degrees)
                hoodMotor.setControl(hoodControlRequest.withPosition(tuningHoodAngle.get().degrees))
            }
            TurretState.PIDTuning -> {
                if (OI.turretPIDTuning())
                    turretMotor.setControl(turretControlRequest.withPosition(15.0.degrees))
                else turretMotor.setControl(CoastOut())
                if (OI.hoodPIDTuning())
                    hoodMotor.setControl(hoodControlRequest.withPosition(15.0.degrees))
                else hoodMotor.setControl(CoastOut())
                Logger.recordOutput("shooter/pid_hoodAngle", hoodAngle.degrees)
                Logger.recordOutput("shooter/pid_hoodTarget", 15.0)
                Logger.recordOutput("shooter/pid_turretAngle", turretAngle.degrees)
                Logger.recordOutput("shooter/pid_turretTarget", 15.0)
            }
            TurretState.SysIdIdle -> turretMotor.setControl(CoastOut())
            TurretState.Aiming -> {
                // convert from 0-360 space to absolute turret motor value.
                // Use the current target position to handle calculations while unwrapping
                val targetAngle =
                    turretCalculator.calculateNewHeadingDegrees(
                        turretControlRequest.Position.rotations,
                        (shotVector.turretAngle - SwerveDrivetrain.state.Pose.rotation).measure,
                    )

                turretMotor.setControl(turretControlRequest.withPosition(targetAngle))

                hoodMotor.setControl(hoodControlRequest.withPosition(shotVector.params.hoodAngle))

                Logger.recordOutput("shooter/hubActiveToShoot", willHubBeActiveToShoot(shotVector))
                Logger.recordOutput("shooter/timeOfFlight", shotVector.params.tof)

                Logger.recordOutput("shooter/turret_turretAngle.degrees", turretAngle.degrees)
                Logger.recordOutput(
                    "shooter/turret_turretControlRequest.positionMeasure.degrees",
                    turretControlRequest.positionMeasure.degrees,
                )
                Logger.recordOutput("shooter/turret_targetAngle.degrees", targetAngle.degrees)
                Logger.recordOutput("shooter/turret_turretAtPosition", turretAtPosition)

                Logger.recordOutput("shooter/hood_hoodAngle.degrees", hoodAngle.degrees)
                Logger.recordOutput(
                    "shooter/hood_shotVector.params.hoodAngle",
                    shotVector.params.hoodAngle,
                )
                Logger.recordOutput("shooter/hood_hoodAtPosition", hoodAtPosition)
            }
            TurretState.FixedHub -> {
                turretMotor.setControl(
                    turretControlRequest.withPosition(Constants.FIXED_HUB_SHOT_TURRET)
                )
                hoodMotor.setControl(hoodControlRequest.withPosition(Constants.FIXED_HUB_SHOT_HOOD))
            }
            TurretState.FixedFeed -> {
                turretMotor.setControl(
                    turretControlRequest.withPosition(Constants.FIXED_FEED_SHOT_TURRET)
                )
                hoodMotor.setControl(
                    hoodControlRequest.withPosition(Constants.FIXED_FEED_SHOT_HOOD)
                )
            }
            TurretState.SysIdQuasistaticForward -> {
                SignalLogger.writeString(
                    "turret_state",
                    SysIdRoutineLog.State.kQuasistaticForward.toString(),
                )
                if (sysIdTimer.hasElapsed(10.0)) {
                    turretMotor.setVoltage(0.0)
                    turretSysIdLog.recordState(SysIdRoutineLog.State.kNone)
                } else {
                    turretMotor.setVoltage(sysIdTimer.get() * 1.0)
                    SignalLogger.writeString(
                        "turret_state",
                        SysIdRoutineLog.State.kQuasistaticForward.toString(),
                    )
                }
            }

            TurretState.SysIdQuasistaticReverse -> {
                if (sysIdTimer.hasElapsed(10.0)) {
                    turretMotor.setVoltage(0.0)
                    turretSysIdLog.recordState(SysIdRoutineLog.State.kNone)
                } else {
                    turretMotor.setVoltage(sysIdTimer.get() * -1.0)
                    SignalLogger.writeString(
                        "turret_state",
                        SysIdRoutineLog.State.kQuasistaticReverse.toString(),
                    )
                }
            }

            TurretState.SysIdDynamicForward -> {
                if (sysIdTimer.hasElapsed(4.0)) {
                    turretMotor.setVoltage(0.0)
                    turretSysIdLog.recordState(SysIdRoutineLog.State.kNone)
                } else {
                    turretMotor.setVoltage(10.0)
                    SignalLogger.writeString(
                        "turret_state",
                        SysIdRoutineLog.State.kDynamicForward.toString(),
                    )
                }
            }

            TurretState.SysIdDynamicReverse -> {
                if (sysIdTimer.hasElapsed(4.0)) {
                    turretMotor.setVoltage(0.0)
                    SignalLogger.writeString("turret_state", SysIdRoutineLog.State.kNone.toString())
                } else {
                    turretMotor.setVoltage(-3.0)
                    SignalLogger.writeString(
                        "turret_state",
                        SysIdRoutineLog.State.kDynamicReverse.toString(),
                    )
                }
            }
        }

        // Flywheel per-state actions
        when (flywheelState) {
            FlywheelState.ShotTuning -> {
                Logger.recordOutput("shooter/shotTuningFlywheelSpeedRPM", flywheelVelocity.rpm)
                flywheelMotor.setControl(
                    flywheelControlRequest.withVelocity(tuningFlywheelSpeed.get().rpm)
                )
            }
            FlywheelState.PIDTuning -> {
                if (OI.flywheelPIDTuning() && flywheelVelocity < 1000.rpm)
                    flywheelMotor.setControl(flywheelControlRequest.withVelocity(1000.rpm))
                else flywheelMotor.setControl(VoltageOut(0.volts))
                Logger.recordOutput("shooter/pid_flywheelTarget", 1000)
                Logger.recordOutput("shooter/pid_flywheelSpeed", flywheelVelocity.rpm)
            }
            FlywheelState.SysIdIdle -> flywheelMotor.setControl(CoastOut())
            FlywheelState.Still -> {
                flywheelMotor.setControl(VoltageOut(0.0))
            }
            FlywheelState.Clearing,
            FlywheelState.Firing -> {
                var speed =
                    when (turretState) {
                        TurretState.FixedHub -> Constants.FIXED_HUB_SHOT_FLYWHEEL
                        TurretState.FixedFeed -> Constants.FIXED_FEED_SHOT_FLYWHEEL
                        else -> if (robotInOpponentAllianceZone) 3000.rpm else shotVector.params.rpm
                    }
                speed *= 1.15
                flywheelMotor.setControl(flywheelControlRequest.withVelocity(speed))
                Logger.recordOutput("shooter/flywheel_shotVector.params.rpm.rpm", speed.rpm)
                Logger.recordOutput(
                    "shooter/flywheel_flywheelMotor.velocity.value.rpm",
                    flywheelVelocity.rpm,
                )
                Logger.recordOutput(
                    "shooter/flywheel_flywheelControlRequest.Velocity.rpm",
                    flywheelControlRequest.Velocity.rpm,
                )
                Logger.recordOutput("shooter/flywheel_flywheelAtSpeed", flywheelAtSpeed)
            }

            FlywheelState.SysIdQuasistaticForward -> {
                if (sysIdTimer.hasElapsed(10.0)) {
                    flywheelMotor.setVoltage(0.0)
                    flywheelSysIdLog.recordState(SysIdRoutineLog.State.kNone)
                } else {
                    flywheelMotor.setVoltage(sysIdTimer.get() * 1.0)
                    SignalLogger.writeString(
                        "flywheel_state",
                        SysIdRoutineLog.State.kQuasistaticForward.toString(),
                    )
                }
            }

            FlywheelState.SysIdQuasistaticReverse -> {
                if (sysIdTimer.hasElapsed(10.0)) {
                    flywheelMotor.setVoltage(0.0)
                    flywheelSysIdLog.recordState(SysIdRoutineLog.State.kNone)
                } else {
                    flywheelMotor.setVoltage(sysIdTimer.get() * -1.0)
                    SignalLogger.writeString(
                        "flywheel_state",
                        SysIdRoutineLog.State.kQuasistaticReverse.toString(),
                    )
                }
            }

            FlywheelState.SysIdDynamicForward -> {
                if (sysIdTimer.hasElapsed(4.0)) {
                    flywheelMotor.setVoltage(0.0)
                    flywheelSysIdLog.recordState(SysIdRoutineLog.State.kNone)
                } else {
                    flywheelMotor.setVoltage(3.0)
                    SignalLogger.writeString(
                        "flywheel_state",
                        SysIdRoutineLog.State.kDynamicForward.toString(),
                    )
                }
            }

            FlywheelState.SysIdDynamicReverse -> {
                if (sysIdTimer.hasElapsed(4.0)) {
                    flywheelMotor.setVoltage(0.0)
                    SignalLogger.writeString(
                        "flywheel_state",
                        SysIdRoutineLog.State.kNone.toString(),
                    )
                } else {
                    flywheelMotor.setVoltage(-3.0)
                    SignalLogger.writeString(
                        "flywheel_state",
                        SysIdRoutineLog.State.kDynamicReverse.toString(),
                    )
                }
            }
        }
        Logger.recordOutput("shooter/sysIdTimer", sysIdTimer.get())
    }

    fun stateTransitions() {
        // Turret transitions
        when (turretState) {
            TurretState.Disabled -> Unit
            TurretState.PIDTuning -> Unit
            TurretState.ShotTuning -> Unit
            TurretState.Aiming -> {
                // not using week 0
                if (OI.overrideFixedHub) turretState = TurretState.FixedHub
                else if (OI.overrideFixedFeed) turretState = TurretState.FixedFeed
            }

            TurretState.FixedHub -> {
                if (OI.overrideFixedFeed) turretState = TurretState.FixedFeed
                else if (!OI.overrideFixedHub) turretState = TurretState.Aiming
            }

            TurretState.FixedFeed -> {
                if (OI.overrideFixedHub) turretState = TurretState.FixedHub
                else if (!OI.overrideFixedFeed) turretState = TurretState.Aiming
            }

            TurretState.SysIdIdle -> {
                if (OI.turretSysIdPressed()) {
                    SignalLogger.start()
                    sysIdTimer.restart()
                    turretState = TurretState.SysIdQuasistaticForward
                }
            }

            TurretState.SysIdQuasistaticForward -> {
                if (OI.turretSysIdReleased()) {
                    SignalLogger.stop()
                    turretState = TurretState.SysIdIdle
                } else if (sysIdTimer.hasElapsed(11.0)) {
                    turretState = TurretState.SysIdQuasistaticReverse
                    sysIdTimer.restart()
                }
            }

            TurretState.SysIdQuasistaticReverse -> {
                if (OI.turretSysIdReleased()) {
                    SignalLogger.stop()
                    turretState = TurretState.SysIdIdle
                } else if (sysIdTimer.hasElapsed(11.0)) {
                    Logger.recordOutput("shooter/quasiReverseOver", sysIdTimer.hasElapsed(11.0))
                    turretState = TurretState.SysIdDynamicForward
                    sysIdTimer.restart()
                }
            }

            TurretState.SysIdDynamicForward -> {
                // If button is released, stop the test
                if (OI.turretSysIdReleased()) {
                    SignalLogger.stop()
                    turretState = TurretState.SysIdIdle
                } else if (sysIdTimer.hasElapsed(5.0)) {
                    turretState = TurretState.SysIdDynamicReverse
                    sysIdTimer.restart()
                }
            }

            TurretState.SysIdDynamicReverse -> {
                // If button is released, stop the test
                if (OI.turretSysIdReleased()) {
                    SignalLogger.stop()
                    turretState = TurretState.SysIdIdle
                } else if (sysIdTimer.hasElapsed(5.0)) {
                    turretState = TurretState.SysIdIdle
                    sysIdTimer.stop()
                    SignalLogger.stop()
                }
            }
        }

        // Flywheel transitions
        when (flywheelState) {
            FlywheelState.PIDTuning,
            FlywheelState.ShotTuning -> {}
            FlywheelState.Still -> {
                if (OI.isShootingHold()) {
                    flywheelState = FlywheelState.Firing
                }
            }

            FlywheelState.Clearing -> {
                if (OI.isShootingHold()) flywheelState = FlywheelState.Firing
                else if (clearTimer.hasElapsed(0.5)) {
                    flywheelState = FlywheelState.Still
                }
            }

            FlywheelState.Firing -> {
                if (!OI.isShootingHold()) {
                    clearTimer.restart()
                    flywheelState = FlywheelState.Clearing
                }
            }

            FlywheelState.SysIdIdle -> {
                flywheelMotor.setControl(idleRequest)
                if (OI.shooterSysIdPressed()) {
                    SignalLogger.start()
                    sysIdTimer.restart()
                    flywheelState = FlywheelState.SysIdQuasistaticForward
                }
            }

            FlywheelState.SysIdQuasistaticForward -> {
                // If button is released, stop the test
                if (OI.shooterSysIdReleased()) {
                    SignalLogger.stop()
                    flywheelState = FlywheelState.SysIdIdle
                } else if (sysIdTimer.hasElapsed(11.0)) {
                    flywheelState = FlywheelState.SysIdQuasistaticReverse
                    sysIdTimer.restart()
                }
            }

            FlywheelState.SysIdQuasistaticReverse -> {
                // If button is released, stop the test
                if (OI.shooterSysIdReleased()) {
                    SignalLogger.stop()
                    flywheelState = FlywheelState.SysIdIdle
                } else if (sysIdTimer.hasElapsed(11.0)) {
                    flywheelState = FlywheelState.SysIdDynamicForward
                    sysIdTimer.restart()
                }
            }

            FlywheelState.SysIdDynamicForward -> {
                // If button is released, stop the test
                if (OI.shooterSysIdReleased()) {
                    SignalLogger.stop()
                    flywheelState = FlywheelState.SysIdIdle
                } else if (sysIdTimer.hasElapsed(5.0)) {
                    flywheelState = FlywheelState.SysIdDynamicReverse
                    sysIdTimer.restart()
                }
            }

            FlywheelState.SysIdDynamicReverse -> {
                // If button is released, stop the test
                if (OI.shooterSysIdReleased()) {
                    SignalLogger.stop()
                    flywheelState = FlywheelState.SysIdIdle
                } else if (sysIdTimer.hasElapsed(5.0)) {
                    flywheelState = FlywheelState.SysIdIdle
                    sysIdTimer.stop()
                    SignalLogger.stop()
                }
            }
        }
    }

    // -----------------------------------------------------
    //                  Utility Functions
    // -----------------------------------------------------
    /** True if your hub will be active when a shot fired now actually gets there */
    private fun willHubBeActiveToShoot(shotVector: ShotVector): Boolean {
        val timeToScore = shotVector.params.tof.seconds + 0.5

        // Score during current active period or within 3 sec of it ending
        //          OR
        // Hub will still be active in next the period
        return if (isHubActive) timeToScore < (timeRemainingInPeriod + 3.0) || isNextPeriodActive
        // Shoot during inactive period but period will switch over to active before it gets counted
        //          OR
        // Shot will get counted within 3 seconds of an inactive period starting
        else (timeToScore > timeRemainingInPeriod) || (timeToScore + timeElapsedInPeriod < 3.0)
    }

    /**
     * Checks whether the robot currently has a valid shot based on the following criteria
     * - Turret, Flywheel, and Hood are within threshold of their desired setpoints
     * - Our hub will be active to count the shot (HUB shot only)
     * - Nets and hub structures are not obstructing the line of fire (FEED shot only)
     */
    var hasValidShot = false

    private fun hasValidShot(): Boolean {
        val shooterTranslation = getFieldRelativeShooterTranslation()
        val isHubShot = SwerveDrivetrain.robotInOwnAllianceZone
        val mechanismsReady = turretAtPosition && flywheelAtSpeed && hoodAtPosition

        val hasValidShot =
            if (isHubShot) {
                mechanismsReady && (willHubBeActiveToShoot(shotVector) || OI.overridePeriodTimer)
            } else {
                val targetPose =
                    SwerveDrivetrain.state.Pose.nearest(listOf(FEED_RIGHT_POSE, FEED_LEFT_POSE))
                val obstructed =
                    segmentsIntersect(
                        shooterTranslation,
                        targetPose.translation,
                        BLUE_HUB_CORNER_1,
                        BLUE_HUB_CORNER_2,
                    ) ||
                        segmentsIntersect(
                            shooterTranslation,
                            targetPose.translation,
                            RED_HUB_CORNER_1,
                            RED_HUB_CORNER_2,
                        )
                mechanismsReady && !obstructed
            }
        Logger.recordOutput("shooter/hasValidShot", hasValidShot)
        //        return flywheelAtSpeed
        return turretAtPosition
    }

    /**
     * Checks if the line segments p1p2 and p3p4 intersect using the cross-product orientation test
     */
    private fun segmentsIntersect(
        p1: Translation2d,
        p2: Translation2d,
        p3: Translation2d,
        p4: Translation2d,
    ): Boolean {
        // Calculates whether going o->a->b is a left/CCW turn (positive) or a right/CW turn
        // (negative)
        fun orientation(o: Translation2d, a: Translation2d, b: Translation2d): Double =
            (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)

        val d1 = orientation(p3, p4, p1)
        val d2 = orientation(p3, p4, p2)
        val d3 = orientation(p1, p2, p3)
        val d4 = orientation(p1, p2, p4)

        // Lines intersect if p1 and p2 are on opposite sides of p3p4 and vice versa is also true
        return (d1 * d2 < 0) && (d3 * d4 < 0)
    }

    /** Returns the position of the shooter in field relative coordinates */
    fun getFieldRelativeShooterTranslation(): Translation2d =
        SwerveDrivetrain.state.Pose.transformBy(
                Transform2d(Constants.SHOOTER_OFFSET, SwerveDrivetrain.state.Pose.rotation)
            )
            .translation

    /** Current shot parameteres, accounting for velocity, distance, etc */
    var shotVector = calculateShotVector()

    private fun calculateShotVector(): ShotVector {
        val robotPose = SwerveDrivetrain.state.Pose
        val fieldRelativeSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(SwerveDrivetrain.state.Speeds, robotPose.rotation)
        val fieldRelativeShooterVelocity =
            getShooterCentricVelocityFromChassisSpeeds(fieldRelativeSpeeds, robotPose)
        val fieldRelativeShooterTranslation = getFieldRelativeShooterTranslation()
        val inAllianceZone = SwerveDrivetrain.robotInOwnAllianceZone
        val targetPose =
            when {
                inAllianceZone -> HUB_POSE
                else -> robotPose.nearest(listOf(FEED_RIGHT_POSE, FEED_LEFT_POSE))
            }
        val controller =
            when {
                inAllianceZone -> hubShotController
                else -> feedShotController
            }
        return controller.calculate(
            fieldRelativeShooterTranslation,
            fieldRelativeShooterVelocity,
            targetPose.translation,
            .05,
        )
    }

    fun getShooterCentricVelocityFromChassisSpeeds(
        robotSpeeds: ChassisSpeeds,
        robotPose: Pose2d,
    ): Translation2d {
        val fieldRelativeOffset = Constants.SHOOTER_OFFSET.rotateBy(robotPose.rotation)

        return Translation2d(
            robotSpeeds.vxMetersPerSecond -
                robotSpeeds.omegaRadiansPerSecond * fieldRelativeOffset.y,
            robotSpeeds.vyMetersPerSecond +
                robotSpeeds.omegaRadiansPerSecond * fieldRelativeOffset.x,
        )
    }

    val turretAngle: Angle
        get() = turretMotor.position.value

    val hoodAngle: Angle
        get() = hoodMotor.position.value

    val flywheelVelocity: AngularVelocity
        get() = flywheelMotor.velocity.value

    val turretAtPosition
        get() = abs(turretMotor.closedLoopError.value) < Constants.TURRET_ANGLE_THRESHOLD

    val flywheelAtSpeed
        get() =
            if (RobotBase.isSimulation()) {
                true
            } else {
                // closed loop error is negative if the measurement is above the target. We ignore
                // threshold in this direction, only worry about it being less than target
                flywheelVelocityDebouncer.calculate(
                    //                    flywheelMotor.closedLoopError.value <
                    // Constants.FLYWHEEL_VELOCITY_THRESHOLD
                    abs(flywheelMotor.velocity.valueAsDouble) >
                        (abs(flywheelMotor.closedLoopReference.valueAsDouble) -
                            Constants.FLYWHEEL_VELOCITY_THRESHOLD)
                )
            }

    val hoodAtPosition
        get() = abs(hoodMotor.closedLoopError.value) < Constants.HOOD_ANGLE_THRESHOLD

    // -----------------------------------------------------
    //                  Simulation Stuff
    // -----------------------------------------------------
    val turretSim =
        DCMotorSim(
            LinearSystemId.createDCMotorSystem(7.8416 / (2 * PI), 0.52511 / (2 * PI)),
            DCMotor.getKrakenX44Foc(1).withReduction(Constants.TURRET_GEAR_RATIO),
        )
    val hoodSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1),
            Constants.HOOD_GEAR_RATIO,
            111.8.poundSquareInches.kilogramSquareMeters,
            hypot(5.89.inches.meters, 4.448.inches.meters),
            0.0,
            40.degrees.radians,
            false,
            0.0,
        )

    fun simulationInit() {
        turretMotor.simState.setMotorType(TalonFXSimState.MotorType.KrakenX44)
        hoodMotor.simState.setMotorType(TalonFXSimState.MotorType.KrakenX44)
    }

    fun simulationPeriodic() {
        turretMotor.simState.setSupplyVoltage(RobotController.getBatteryVoltage())
        turretSim.inputVoltage = turretMotor.simState.motorVoltage
        turretSim.update(0.02)
        turretMotor.simState.setRawRotorPosition(
            turretSim.angularPositionRotations * Constants.TURRET_GEAR_RATIO
        )
        turretMotor.simState.setRotorVelocity(
            turretSim.angularVelocityRPM * Constants.TURRET_GEAR_RATIO / 60.0
        )

        hoodSim.setInputVoltage(hoodMotor.simState.motorVoltage)
        hoodSim.update(0.02)
        hoodMotor.simState.setRawRotorPosition(
            Units.radiansToRotations(hoodSim.angleRads) * Constants.HOOD_GEAR_RATIO
        )
        hoodMotor.simState.setRotorVelocity(
            Units.radiansToRotations(hoodSim.velocityRadPerSec) * Constants.HOOD_GEAR_RATIO
        )
    }

    private var lastShotTime = -1.0
    private var lastHubTime = -1.0
    private val shotTimePub =
        NetworkTableInstance.getDefault().getDoubleTopic("shooter/shotTime").publish()

    private val isDetected = hubCanRange.isDetected
    private val shooterVelocity = flywheelMotor.velocity

    fun measureShotTime() {
        if (!hubCanRange.isConnected) return
        BaseStatusSignal.waitForAll(0.02, isDetected, shooterVelocity)
        if (isDetected.value) {
            lastHubTime = isDetected.timestamp.time
        }
        if (shooterVelocity.valueAsDouble < 24.0) {
            lastShotTime = shooterVelocity.timestamp.time
        }
        shotTimePub.set(lastHubTime - lastShotTime)
    }
}
