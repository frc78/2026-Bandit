package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import frc.robot.OI
import frc.robot.generated.TunerConstants
import frc.robot.lib.*
import frc.robot.subsystems.Climber.Constants.GEAR_RATIO
import frc.robot.subsystems.Climber.Constants.SPOOL_CIRCUMFERENCE
import kotlin.math.PI
import org.littletonrobotics.junction.Logger

object Climber {

    object Constants {
        const val GEAR_RATIO = (80.0 * 80.0) / (18.0 * 12.0)
        val SPOOL_CIRCUMFERENCE = 0.5.inches.meters * PI
        val CLIMBED_ANGLE = (-3.75).rotations
        val UNCLIMBED_ANGLE = 0.rotations
    }

    enum class ClimberState {
        Unclimbed,
        Climbed,
        ManualUp,
        ManualDown,
    }

    var currentState = ClimberState.Unclimbed
        set(newState) {
            // Manual climber control should only be used to reset it to its most extended state
            // between matches
            // Thus, when switching away from manual mode, reset the climber to pose 0
            if (currentState == ClimberState.ManualUp || currentState == ClimberState.ManualDown)
                climberMotor.setPosition(0.rotations)
            field = newState
        }

    private val UnclimbRequest = PositionVoltage(Constants.UNCLIMBED_ANGLE)
    private val ClimbRequest = PositionVoltage(Constants.CLIMBED_ANGLE)

    // Manual "up" and "down" refer to the climber spool spinning up or down when viewed from the
    // side of the robot
    private val ManualUpRequest = VoltageOut(1.5.volts)
    private val ManualDownRequest = VoltageOut((-1.5).volts)

    private val climberMotor =
        TalonFX(40, TunerConstants.kCANBus).apply {
            configurator.apply(
                TalonFXConfiguration().apply {
                    Feedback.SensorToMechanismRatio = GEAR_RATIO
                    Slot0.apply { kP = 200.0 }
                    MotorOutput.NeutralMode = NeutralModeValue.Brake
                }
            )
            setPosition(0.rotations)
        }

    fun stateActions() {
        Logger.recordOutput("climber/state", currentState)
        Logger.recordOutput("climber/angle", climberAngle)
        Logger.recordOutput("climber/statorCurrentAmps", climberMotor.statorCurrent.value.amps)
        when (currentState) {
            ClimberState.Unclimbed -> climberMotor.setControl(UnclimbRequest)
            ClimberState.Climbed -> climberMotor.setControl(ClimbRequest)
            ClimberState.ManualUp -> climberMotor.setControl(ManualUpRequest)
            ClimberState.ManualDown -> climberMotor.setControl(ManualDownRequest)
        }
    }

    fun stateTransitions() {
        when (currentState) {
            // NOTE: manual toggling between Climbed and Unclimbed is handled in OI.kt when the
            // driver start button is pressed.  Doing it there instead of here prevents the state
            // from getting constantly set based on the toggle and allows us to programmatically
            // control the climber state based on match factors.
            ClimberState.Unclimbed -> {
                if (OI.manualClimberUp()) currentState = ClimberState.ManualUp
                else if (OI.manualClimberDown()) currentState = ClimberState.ManualDown
            }
            ClimberState.Climbed -> {
                if (SwerveDrivetrain.currentState == SwerveDrivetrain.SwerveState.AlignToTower)
                    currentState = ClimberState.Unclimbed
            }
            // Assume when leaving manual control that the climber is all the way up
            ClimberState.ManualUp -> {
                if (!OI.manualClimberUp()) currentState = ClimberState.Unclimbed
            }
            ClimberState.ManualDown -> {
                if (!OI.manualClimberDown()) currentState = ClimberState.Unclimbed
            }
        }
    }

    val climberHeight
        get() = climberMotor.position.valueAsDouble * SPOOL_CIRCUMFERENCE

    val climberAngle
        get() = climberMotor.position.value

    val climberSim =
        ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            GEAR_RATIO,
            135.pounds.kilograms,
            .5.inches.meters,
            -6.inches.meters,
            0.inches.meters,
            false,
            0.inches.meters,
        )

    fun simulationPeriodic() {
        climberSim.setInputVoltage(climberMotor.simState.motorVoltage)
        climberSim.update(0.02)
        climberMotor.simState.setRawRotorPosition(
            climberSim.positionMeters / SPOOL_CIRCUMFERENCE * GEAR_RATIO
        )
        climberMotor.simState.setRotorVelocity(
            climberSim.velocityMetersPerSecond / SPOOL_CIRCUMFERENCE * GEAR_RATIO
        )
    }
}
