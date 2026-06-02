package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.CoastOut
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.wpilibj.Timer
import frc.robot.generated.TunerConstants
import frc.robot.lib.rotationsPerSecond
import frc.robot.lib.volts
import frc.robot.subsystems.Indexer.Constants.CANRANGE_FOV_RANGE_X
import frc.robot.subsystems.Indexer.Constants.CANRANGE_FOV_RANGE_Y
import frc.robot.subsystems.Indexer.Constants.CANRANGE_ID
import frc.robot.subsystems.Indexer.Constants.CANRANGE_PROXIMITY_THRESHOLD
import frc.robot.subsystems.Indexer.Constants.FEEDER_FORWARD_SPEED
import frc.robot.subsystems.Indexer.Constants.SPINDEXER_FORWARD_VOLTAGE
import frc.robot.subsystems.Indexer.Constants.SPINDEXER_REVERSE_VOLTAGE
import frc.robot.subsystems.Indexer.Constants.SPINDEXER_SLOW_VOLTAGE
import frc.robot.subsystems.Indexer.IndexerState.*
import frc.robot.subsystems.Shooter.FlywheelState
import org.littletonrobotics.junction.Logger

object Indexer {

    private object Constants {
        val SPINDEXER_FORWARD_VOLTAGE = 9.0.volts
        val SPINDEXER_REVERSE_VOLTAGE = (-9.0).volts
        val SPINDEXER_SLOW_VOLTAGE = 5.0.volts
        val FEEDER_FORWARD_SPEED = 100.rotationsPerSecond

        const val CANRANGE_ID = 32
        const val CANRANGE_PROXIMITY_THRESHOLD = 0.04
        const val CANRANGE_FOV_RANGE_X = 10.0
        const val CANRANGE_FOV_RANGE_Y = 10.0
    }

    enum class IndexerState {
        Still,
        Indexing,
        WaitingToShoot,
        Clearing,
    }

    private val SpindexerForwardRequest = VoltageOut(SPINDEXER_FORWARD_VOLTAGE)
    private val SpindexerForwardRequestSlow = VoltageOut(SPINDEXER_SLOW_VOLTAGE)
    private val SpindexerReverseRequest = VoltageOut(SPINDEXER_REVERSE_VOLTAGE)
    private val FeederForwardRequest = VelocityTorqueCurrentFOC(FEEDER_FORWARD_SPEED)
    private val IdleRequest = CoastOut()

    private val fuelSensor =
        CANrange(CANRANGE_ID, TunerConstants.kCANBus).apply {
            val config =
                CANrangeConfiguration().apply {
                    ProximityParams.ProximityThreshold = CANRANGE_PROXIMITY_THRESHOLD
                    FovParams.FOVRangeY = CANRANGE_FOV_RANGE_Y
                    FovParams.FOVRangeX = CANRANGE_FOV_RANGE_X
                }
            configurator.apply(config)
        }

    private val spindexerMotor =
        TalonFX(30, TunerConstants.kCANBus).apply {
            val config =
                TalonFXConfiguration().apply {
                    MotorOutput.apply { Inverted = InvertedValue.CounterClockwise_Positive }
                }
            configurator.apply(config)
        }
    private val feederMotor =
        TalonFX(31, TunerConstants.kCANBus).apply {
            val config =
                TalonFXConfiguration().apply {
                    MotorOutput.apply { this.Inverted = InvertedValue.Clockwise_Positive }
                    Slot0.kP = 999.0
                    TorqueCurrent.PeakReverseTorqueCurrent = 0.0
                }

            configurator.apply(config)
        }

    var currentState = Still

    private val directionChangeTimer = Timer()

    enum class SpinDirection {
        Forward,
        Reverse,
    }

    private var spinState = SpinDirection.Reverse

    fun switchDirection() {
        spinState =
            when (spinState) {
                SpinDirection.Forward -> SpinDirection.Reverse
                SpinDirection.Reverse -> SpinDirection.Forward
            }
    }

    fun stateActions() {
        Logger.recordOutput("indexer/state", currentState)
        Logger.recordOutput("indexer/spinState", spinState)
        Logger.recordOutput("indexer/supplyCurrent", spindexerMotor.supplyCurrent.value)
        Logger.recordOutput("indexer/statorCurrent", spindexerMotor.statorCurrent.value)
        Logger.recordOutput("indexer/torqueCurrent", spindexerMotor.torqueCurrent.value)
        when (currentState) {
            Still -> {
                spindexerMotor.setControl(IdleRequest)
                feederMotor.setControl(IdleRequest)
            }

            Clearing -> {
                spindexerMotor.setControl(IdleRequest)
                feederMotor.setControl(FeederForwardRequest)
            }

            Indexing -> {
                //                if (spinState == SpinDirection.Forward) {
                //                    spindexerMotor.setControl(SpindexerForwardRequest)
                //                } else {
                //                    spindexerMotor.setControl(SpindexerReverseRequest)
                //                }
                //                if (fuelSensor.isDetected.value) {
                //                    directionChangeTimer.restart()
                //                } else if (directionChangeTimer.hasElapsed(0.5)) {
                //                    switchDirection()
                //                    directionChangeTimer.restart()
                //                }

                //                // Fuel detected, keep going!
                //                if (fuelSensor.isDetected.value) {
                //                    directionChangeTimer.restart()
                //                    spindexerMotor.setControl(SpindexerReverseRequest)
                //                }
                //                // No fuel detected in 0.75 sec
                //                else if (directionChangeTimer.hasElapsed(0.75)) {
                //                    // Reverse direction for 0.5 sec to unjam
                //                    if (!directionChangeTimer.hasElapsed(1.25))
                //                        spindexerMotor.setControl(SpindexerForwardRequest)
                //                    // After reversing 0.5 sec, reset timer and try again
                //                    else {
                //                        directionChangeTimer.restart()
                //                        spindexerMotor.setControl(SpindexerReverseRequest)
                //                    }
                //                }
                //                // No fuel right now but detected within 0.75 sec, keep going!
                //                else spindexerMotor.setControl(SpindexerReverseRequest)
                spindexerMotor.setControl(SpindexerReverseRequest)
                feederMotor.setControl(FeederForwardRequest)
            }
            WaitingToShoot -> {
                spindexerMotor.setControl(SpindexerForwardRequestSlow)
                feederMotor.setControl(IdleRequest)
            }
        }
    }

    fun stateTransitions() {
        when (currentState) {
            Still -> {
                if (Shooter.flywheelState == FlywheelState.Firing) {
                    directionChangeTimer.restart()
                    currentState = WaitingToShoot
                }
            }

            Clearing -> {
                if (Shooter.flywheelState != FlywheelState.Clearing) {
                    currentState = Still
                }
            }

            Indexing -> {
                if (Shooter.flywheelState != FlywheelState.Firing) {
                    currentState = Still
                } else if (!Shooter.hasValidShot) {
                    currentState = WaitingToShoot
                }
            }

            WaitingToShoot -> {
                if (Shooter.flywheelState != FlywheelState.Firing) {
                    currentState = Still
                } else if (Shooter.hasValidShot) {
                    directionChangeTimer.restart()
                    currentState = Indexing
                }
            }
        }
    }

    /** The times at which fuel was detected by the sensor, in seconds. */
    private val fuelTimes = mutableListOf<Double>()
    private var fuelCount = 0
    private var fuelPresentLatch = false

    fun countFuel() {
        // Update fuel times and calculate bps. We use a latch to only record the time at which fuel
        // is first detected, not continuously while it's detected.
        if (fuelPresentLatch) {
            if (!fuelSensor.isDetected.value) {
                fuelPresentLatch = false
            }
        } else {
            if (fuelSensor.isDetected.value) {
                fuelTimes.add(fuelSensor.isDetected.timestamp.time)
                fuelCount++
                fuelPresentLatch = true
            }
        }
    }

    fun periodic() {
        if (fuelTimes.size >= 2) {
            Logger.recordOutput("feeder/bps_2", calculateBps(fuelTimes.takeLast(2)))
        }
        if (fuelTimes.size >= 5) {
            Logger.recordOutput("feeder/bps_5", calculateBps(fuelTimes.takeLast(5)))
        }
        if (fuelTimes.size >= 10) {
            Logger.recordOutput("feeder/bps_10", calculateBps(fuelTimes.takeLast(10)))
        }
        Logger.recordOutput("feeder/fuel_count", fuelCount)
    }

    private fun calculateBps(fuel: List<Double>): Double {
        val duration = fuel.last() - fuel.first()
        return if (duration > 0) (fuel.size - 1) / duration else 0.0
    }
}
