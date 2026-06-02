package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.CoastOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.OI
import frc.robot.generated.TunerConstants
import frc.robot.lib.*
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.math.sin
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

object Intake {

    object Constants {
        const val PIVOT_GEAR_RATIO = 66.0 * 72.0 / (12.0 * 12.0)
        const val PIVOT_CURRENT_LIMIT = 10.0
        const val ROLLER_GEAR_RATIO = (18.0 * 36.0) / (12.0 * 18.0)
        const val ROLLER_CURRENT_LIMIT = 80.0
        val DEPLOYED_POSITION = 115.0.degrees
        val RETRACTED_POSITION = (20.0).degrees
        val Unjamming_Lower_Position = 90.degrees
        val ROLLER_INTAKE_SPEED = 2000.rpm
        val ROLLER_OUTTAKE_SPEED = (-2000).rpm
        val ROLLER_UNJAM_SPEED = 0.rpm
        val JOSTLE_SPEED = 1333.rpm
        val BASE_OSCILLATION = 60.degrees
    }

    enum class IntakeState {
        /** Intake up and rollers off */
        Retracted,
        /** Intake out, but not running rollers. Used when holding fuel in the hopper */
        Deployed,
        /** Intake out and rollers actively intaking */
        Intaking,
        /** Jostles the intake to help fuel fall into the spindexer */
        Jostle,
        /** Intake out and rollers actively outtaking */
        Outtaking,
        /** Test mode state for running the pivot to a specific position. Not used in competition */
        PIDTuning,
    }

    var currentState = IntakeState.Retracted

    val rollerMotor =
        TalonFX(20, TunerConstants.kCANBus).apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = Constants.ROLLER_GEAR_RATIO
                    CurrentLimits.StatorCurrentLimit = Constants.ROLLER_CURRENT_LIMIT
                    CurrentLimits.SupplyCurrentLimit = 40.0
                    Slot0.apply {
                        kP = 40.0
                        kD = 7.0
                        this.GravityType = GravityTypeValue.Arm_Cosine
                    }
                    MotionMagic.apply {
                        MotionMagicCruiseVelocity = 2.0
                        MotionMagicAcceleration = 1.0
                    }
                }
            )
        }

    private val off = VoltageOut(0.0.volts)
    private val intakeRequest = VelocityVoltage(Constants.ROLLER_INTAKE_SPEED)
    //    private val unjamRequest = VelocityVoltage(Constants.ROLLER_UNJAM_SPEED)
    private val outtakeRequest = VelocityVoltage(Constants.ROLLER_OUTTAKE_SPEED)
    //    private val intakeRequest = VoltageOut(10.0.volts)
    //    private val unjamRequest = VoltageOut(0.0.volts)
    //    private val outtakeRequest = VoltageOut((-10.0).volts)

    val pivotMotor =
        TalonFX(21, TunerConstants.kCANBus).apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = Constants.PIVOT_GEAR_RATIO
                    //                    Feedback.FeedbackRotorOffset = -0.547852
                    CurrentLimits.StatorCurrentLimit = Constants.PIVOT_CURRENT_LIMIT
                    MotionMagic.apply {
                        MotionMagic.MotionMagicCruiseVelocity = 1.0
                        MotionMagicAcceleration = 1.0
                    }
                    Slot0.apply {
                        kP = 225.0
                        kD = 10.0
                        kS = 0.8
                    }
                    CurrentLimits.apply { StatorCurrentLimit = 60.0 }
                }
            )
        }

    val deployedRequest = PositionVoltage(Constants.DEPLOYED_POSITION)
    val retractedRequest = PositionVoltage(Constants.RETRACTED_POSITION)
    val pivotUnjamRequest = PositionVoltage(Constants.DEPLOYED_POSITION)
    val jostleRequest = VelocityVoltage(Constants.JOSTLE_SPEED)

    private val jostleAmplitude = LoggedNetworkNumber("intake/jostle_amplitude", 8.0)
    private val jostleFrequency = LoggedNetworkNumber("intake/jostle_frequency", 5.0)
    private val jostleAverage = LoggedNetworkNumber("intake/jostle_average", 95.0)

    fun stateActions() {
        Logger.recordOutput("intake/state", currentState)
        when (currentState) {
            IntakeState.PIDTuning -> {
                if (OI.intakePIDTuning()) pivotMotor.setControl(PositionVoltage(55.0.degrees))
                else pivotMotor.setControl(CoastOut())
                rollerMotor.setControl(CoastOut())
                Logger.recordOutput("intake/pid_pivotTarget", 55.0)
                Logger.recordOutput("intake/pid_pivotAngle", pivotAngle.degrees)
            }
            IntakeState.Retracted -> {
                pivotMotor.setControl(retractedRequest)
                rollerMotor.setControl(off)
            }
            IntakeState.Deployed -> {
                pivotMotor.setControl(deployedRequest)
                rollerMotor.setControl(off)
            }
            IntakeState.Intaking -> {
                pivotMotor.setControl(deployedRequest)
                rollerMotor.setControl(intakeRequest)
            }
            IntakeState.Jostle -> {
                // Add 60 degree upwards jostle, one up-down cycle every second
                val jostleAngle =
                    jostleAmplitude.get() *
                        sin(
                            2 *
                                PI *
                                jostleFrequency.get() *
                                RobotController.getMeasureTime().seconds
                        ) + jostleAverage.get()
                pivotMotor.setControl(pivotUnjamRequest.withPosition(jostleAngle.degrees))
                rollerMotor.setControl(jostleRequest)
            }
            IntakeState.Outtaking -> {
                pivotMotor.setControl(deployedRequest)
                rollerMotor.setControl(outtakeRequest)
            }
        }
    }

    fun stateTransitions() {
        when (currentState) {
            IntakeState.PIDTuning -> {}
            IntakeState.Retracted -> {
                if (OI.isDeployed) {
                    currentState = IntakeState.Intaking
                }
            }
            IntakeState.Deployed -> {
                if (OI.intakeRollersActive()) {
                    currentState = IntakeState.Intaking
                } else if (OI.jostleIntake()) {
                    currentState = IntakeState.Jostle
                } else if (!OI.isDeployed) {
                    currentState = IntakeState.Retracted
                }
            }
            IntakeState.Intaking -> {
                if (!OI.isDeployed) {
                    currentState = IntakeState.Retracted
                } else if (!OI.intakeRollersActive()) {
                    currentState = IntakeState.Deployed
                } else if (OI.jostleIntake()) {
                    currentState = IntakeState.Jostle
                }
            }
            IntakeState.Jostle -> {
                if (!OI.isDeployed) {
                    currentState = IntakeState.Retracted
                } else if (!OI.jostleIntake()) {
                    // Don't need to remember previous state (intaking or deployed) because the
                    // manipulator needs to press intake button to intake anyway. If you go to
                    // Intaking and button isn't held, it'll go back to Deployed
                    currentState = IntakeState.Deployed
                }
            }
            IntakeState.Outtaking -> {
                // NOT CURRENTLY REACHABLE
                if (OI.isDeployed) currentState = IntakeState.Intaking
            }
        }
    }

    val pivotAngle: Angle
        get() = pivotMotor.position.value

    fun periodic() {
        Logger.recordOutput("intake/pivot_angle", pivotAngle.degrees)
    }

    private val pivotSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1),
            Constants.PIVOT_GEAR_RATIO,
            816.poundSquareInches.kilogramSquareMeters,
            hypot(9.965.inches.meters, .466.inches.meters),
            0.degrees.radians,
            120.degrees.radians,
            false,
            0.degrees.radians,
            0.0,
            0.0,
        )

    fun simulationPeriodic() {
        pivotSim.setInputVoltage(pivotMotor.simState.motorVoltage)
        pivotSim.update(0.02)
        pivotMotor.simState.setRawRotorPosition(
            pivotSim.angleRads.radians * Constants.PIVOT_GEAR_RATIO
        )
        pivotMotor.simState.setRotorVelocity(
            pivotSim.velocityRadPerSec.radiansPerSecond * Constants.PIVOT_GEAR_RATIO
        )
    }
}
