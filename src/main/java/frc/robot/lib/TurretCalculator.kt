package frc.robot.lib

import edu.wpi.first.units.measure.Angle

/**
 * Calculates the closest feasible turret position for a given heading. The turrets real range is
 * minAngle..maxAngle degrees.
 */
class TurretCalculator(private val minAngle: Angle, private val maxAngle: Angle) {

    /**
     * @param current: Turret position in minAngle..maxAngle
     * @param target: Robot-oriented target in 0..360
     * @return Turret position in minAngle..maxAngle
     */
    fun calculateNewHeadingDegrees(current: Angle, target: Angle): Angle {
        // ∆ should always be ±180, which is a range of 360.
        // ∆.mod(360) will return 0-360. Subtract 180 to shift the range.
        // To avoid this shifting the actual result, we add 180 before modding
        val delta = (target - current + 180.degrees).degrees.mod(360.0) - 180
        val unboundedPosition = current + delta.degrees

        return when {
            unboundedPosition > maxAngle -> unboundedPosition - 360.degrees
            unboundedPosition < minAngle -> unboundedPosition + 360.degrees
            else -> unboundedPosition
        }
    }
}
