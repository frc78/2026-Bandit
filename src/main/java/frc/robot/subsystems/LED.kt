package frc.robot.subsystems

import com.ctre.phoenix6.controls.SolidColor
import com.ctre.phoenix6.controls.StrobeAnimation
import com.ctre.phoenix6.hardware.CANdle
import com.ctre.phoenix6.signals.RGBWColor
import edu.wpi.first.wpilibj.RobotState
import frc.robot.OI
import frc.robot.generated.TunerConstants

object LED {
    private val candle = CANdle(1, TunerConstants.kCANBus)

    val GREEN = RGBWColor(0, 255, 0)
    val BLUE = RGBWColor(0, 0, 255)
    val RED = RGBWColor(255, 0, 0)
    val ORANGE = RGBWColor(255, 128, 0)
    val OFF = RGBWColor(0, 0, 0)
    val WHITE = RGBWColor(0, 0, 0)

    val solidColor = SolidColor(0, 7)

    enum class LedState {
        DisabledSeeTag,
        DisabledNoTag,
        Enabled,
    }

    var currentState = LedState.DisabledNoTag

    fun stateMachine() {
        when (currentState) {
            LedState.DisabledNoTag -> {
                candle.setControl(solidColor.withColor(RED))
                if (RobotState.isEnabled()) {
                    currentState = LedState.Enabled
                }
            }
            LedState.DisabledSeeTag -> {
                candle.setControl(solidColor.withColor(GREEN))
                if (RobotState.isEnabled()) {
                    currentState = LedState.Enabled
                }
            }
            LedState.Enabled -> {
                //                // Blink with RSL
                //                if (RobotController.getRSLState()) {
                //                    candle.setControl(solidColor.withColor(ORANGE))
                //                } else {
                //                    candle.setControl(solidColor.withColor(OFF))
                //                }
                if (OI.isShooting) {
                    candle.setControl(StrobeAnimation(0, 7).withColor(GREEN))
                } else {
                    candle.setControl(solidColor.withColor(BLUE))
                }
            }
        }
    }
}
