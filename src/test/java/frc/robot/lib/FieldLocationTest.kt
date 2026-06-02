package frc.robot.lib

import frc.robot.FieldLocation
import kotlin.test.Test
import kotlin.test.assertEquals

class FieldLocationTest {

    private val HUB_POSE by FieldLocation(182.11.inches, 158.84.inches)

    @Test
    fun `hub mirror`() {
        assertEquals((651.22 - 182.11).inches.meters, HUB_POSE.x, .01)
        assertEquals((158.84).inches.meters, HUB_POSE.y, .01)
    }
}
