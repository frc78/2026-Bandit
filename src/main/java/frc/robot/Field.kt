package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import frc.robot.FieldConstants.MIDPOINT
import frc.robot.lib.inches
import frc.robot.lib.meters
import kotlin.reflect.KProperty

object FieldConstants {
    // Field measurements
    val MIDPOINT = Translation2d(325.61.inches, 158.84.inches)

    // AUTO POSES
    val BLUE_AUTO_LINE_X = 156.61.inches.meters
    val RED_AUTO_LINE_X = 494.61.inches.meters

    // Hub structure corners
    //    32.86 in from center of hub to front of net
    val BLUE_HUB_CORNER_1 = Translation2d(214.42.inches, 119.639.inches) // 129.639.inches)
    val BLUE_HUB_CORNER_2 = Translation2d(214.42.inches, 198.049.inches) // 188.049.inches)
    val RED_HUB_CORNER_1 = BLUE_HUB_CORNER_1.rotateAround(MIDPOINT, Rotation2d.k180deg)
    val RED_HUB_CORNER_2 = BLUE_HUB_CORNER_2.rotateAround(MIDPOINT, Rotation2d.k180deg)

    // Shooting targets
    val HUB_POSE by FieldLocation(182.11.inches, 158.84.inches)
    val FEED_LEFT_POSE by FieldLocation(2.meters, 6.07.meters)
    val FEED_RIGHT_POSE by FieldLocation(2.meters, 2.meters)

    // Alignment Poses
    val LEFT_PRE_CLIMB_POSE by FieldLocation(1.07.meters, 4.87.meters, Rotation2d.kCCW_90deg)
    val LEFT_CLIMB_POSE by FieldLocation(1.07.meters, 4.55.meters, Rotation2d.kCCW_90deg)
    val RIGHT_PRE_CLIMB_POSE by FieldLocation(1.15.meters, 2.55.meters, Rotation2d.kCW_90deg)
    val RIGHT_CLIMB_POSE by FieldLocation(1.15.meters, 3.1.meters, Rotation2d.kCW_90deg)
}

class FieldLocation(x: Distance, y: Distance, rotation: Rotation2d = Rotation2d.kZero) {
    constructor(
        xMeters: Double,
        yMeters: Double,
        rotation: Rotation2d = Rotation2d.kZero,
    ) : this(xMeters.meters, yMeters.meters, rotation)

    operator fun getValue(thisRef: Any?, property: KProperty<*>): Pose2d {
        return when (DriverStation.getAlliance().orElse(Red)) {
            Red -> redCoordinate
            else -> blueCoordinate
        }
    }

    val blueCoordinate = Pose2d(x, y, rotation)
    val redCoordinate = blueCoordinate.rotateAround(MIDPOINT, Rotation2d.k180deg)
}
