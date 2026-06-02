package frc.robot.lib.projectile

import edu.wpi.first.math.interpolation.Interpolatable
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.InverseInterpolator
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Time
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.lib.interpolate
import frc.robot.lib.meters
import frc.robot.lib.rpm
import frc.robot.lib.seconds

val HubLut =
    ShooterLUT().apply {
        put(12.0, 1.3, 1200, 1.008)
        put(18.0, 1.3, 1200, 1.058)
        put(24.0, 1.3, 1200, 1.058)
        put(30.0, 1.3, 1260, 1.042)
        put(36.0, 1.3, 1260, 1.125)
        put(42.0, 2.3, 1320, 1.192)
        put(48.0, 2.55, 1320, 1.183)
        put(54.0, 5.54, 1365, 1.192)
        put(60.0, 6.0, 1380, 1.283)
        put(72.0, 7.5, 1410, 1.2)
        put(84.0, 10.02, 1455, 1.175)
        put(96.0, 12.56, 1455, 1.250)
        put(108.0, 15.5, 1485, 1.19) // ToF is a guess
        put(120.0, 19.1, 1500, 0.975)
        put(132.0, 22.0, 1506, 1.117)
        put(144.0, 24.85, 1536, 1.117)
        put(156.0, 22.9, 1569, 1.142)
        put(168.0, 22.0, 1596, 1.183)
        put(180.0, 23.4, 1665, 1.19)
        put(192.0, 26.1, 1734, 1.175)
        put(204.0, 26.8, 1734, 1.192)
        put(216.0, 26.8, 1773, 1.183)
        put(228.0, 28.1, 1818, 1.175)
        put(240.0, 29.45, 1848, 1.2)
    }

val HubLutHighCeiling =
    ShooterLUT().apply { put(0.0, ShotParams(0.0.degrees, 0.0.rpm, 1.0.seconds)) }

val FeedLut =
    ShooterLUT().apply {
        put(0.0, ShotParams(60.degrees, 5000.rpm, 0.3.seconds))
        put(1.016, ShotParams(60.degrees, 5000.rpm, 0.45.seconds))
    }

class ShotParams(val hoodAngle: Angle, val rpm: AngularVelocity, val tof: Time) :
    Interpolatable<ShotParams> {

    override fun interpolate(endValue: ShotParams, t: Double): ShotParams {
        return ShotParams(
            hoodAngle.interpolate(endValue.hoodAngle, t),
            rpm.interpolate(endValue.rpm, t),
            tof.interpolate(endValue.tof, t),
        )
    }
}

/** Lookup Table (LUT) for a shooter with adjustable flywheel speed and hood angle */
class ShooterLUT {
    /** Distance -> Shot Param LUT */
    private val distanceMap =
        InterpolatingTreeMap(InverseInterpolator.forDouble(), ShotParams::interpolate)
    /**
     * Maps a velocity to a distance. This table is populated using the distance and time-of-flight
     * measurements from the distance -> shot param LUT
     */
    private val velocityMap = InterpolatingDoubleTreeMap()

    /**
     * Helper method to accept values from spreadsheet
     *
     * @param distance Recorded distance in inches. This method will automatically add the extra
     *   distance
     * @param angle Angle of hood in degrees
     * @param rpm Angular velocity of flywheel in rpm
     * @param tof Time of flight in seconds
     */
    fun put(distance: Double, angle: Double, rpm: Number, tof: Double) {
        put((distance + 23.5 + 18).inches.meters, ShotParams(angle.degrees, rpm.rpm, tof.seconds))
    }

    fun put(distance: Double, params: ShotParams) {
        distanceMap.put(distance, params)
        velocityMap.put(distance / params.tof.seconds, distance)
    }

    fun getParamsForDistance(distance: Double): ShotParams {
        return distanceMap[distance]
    }

    fun getParamsForVelocity(velocity: Double): ShotParams {
        return distanceMap[velocityMap[velocity]]
    }
}
