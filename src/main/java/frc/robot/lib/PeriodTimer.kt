package frc.robot.lib

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.Logger

object PeriodTimer {

    enum class Period(
        val startTime: Double,
        val duration: Double,
        val activeForAutoWinner: Boolean,
        val activeForAutoLoser: Boolean,
    ) {
        Transition(0.0, 10.0, true, true),
        Shift1(10.0, 25.0, false, true),
        Shift2(35.0, 25.0, true, false),
        Shift3(60.0, 25.0, false, true),
        Shift4(85.0, 25.0, true, false),
        Endgame(110.0, 30.0, true, true),
    }

    private val overrideGameData =
        SendableChooser<Alliance>().apply {
            setDefaultOption("Auto", null)
            addOption("Blue", Alliance.Blue)
            addOption("Red", Alliance.Red)
            SmartDashboard.putData("Won Auto Override", this)
        }

    /** The alliance that won auto based on the game data. Null if no game data */
    val autoWinner: Alliance?
        get() =
            overrideGameData.selected
                ?: when (DriverStation.getGameSpecificMessage()) {
                    "R" -> Alliance.Red
                    "B" -> Alliance.Blue
                    else -> null
                }

    /** True if the robot's alliance won auto. Null if no auto winner */
    val wonAuto
        get() = autoWinner?.let { DriverStation.getAlliance().orElse(Alliance.Red) == it }

    private val teleopTimer = Timer()

    /**
     * Call this function to start the period timer and begin tracking periods in a match.
     *
     * You most likely want to call this in teleopInit
     */
    fun startPeriodTimer() {
        teleopTimer.restart()
    }

    /**
     * Call this method to refresh the current period slice.
     *
     * You most likely want to call this in teleopPeriodic
     */
    fun refresh() {
        Logger.recordOutput("Period/wonAuto", wonAuto == true)
        Logger.recordOutput("Period/current", currentPeriod)
        Logger.recordOutput("Period/time_remaining", timeRemainingInPeriod)
        Logger.recordOutput("Period/alliance_active", isHubActive)
    }

    val currentPeriod: Period
        get() = Period.entries.lastOrNull { it.startTime <= teleopTimer.get() } ?: Period.Transition

    /** True if your hub is active in the next period or if the current period is the last one */
    val isNextPeriodActive
        get() =
            if (currentPeriod == Period.Endgame) true
            else {
                val nextPeriod = Period.entries[Period.entries.indexOf(currentPeriod) + 1]
                wonAuto?.let {
                    if (it) nextPeriod.activeForAutoWinner else nextPeriod.activeForAutoLoser
                } ?: true
            }

    /** True if your hub is active, or if no auto winner */
    val isHubActive
        get() =
            wonAuto?.let {
                if (it) {
                    currentPeriod.activeForAutoWinner
                } else currentPeriod.activeForAutoLoser
            } ?: true

    /** Time remaining in the current period */
    val timeRemainingInPeriod: Double
        get() =
            with(currentPeriod) {
                return maxOf(0.0, startTime + duration - teleopTimer.get())
            }

    /** Time elapsed in the current period */
    val timeElapsedInPeriod: Double
        get() =
            with(currentPeriod) {
                return maxOf(0.0, teleopTimer.get() - startTime)
            }
}
