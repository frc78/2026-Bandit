package frc.robot.lib

import kotlin.test.Test
import kotlin.test.assertEquals

class TurretTest {

    @Test
    fun testValues720() {
        val calc = TurretCalculator(0.degrees, 720.degrees)
        assertEquals(0.degrees, calc.calculateNewHeadingDegrees(0.degrees, 0.degrees))
        assertEquals(180.degrees, calc.calculateNewHeadingDegrees(0.degrees, 180.degrees))
        assertEquals(359.degrees, calc.calculateNewHeadingDegrees(0.degrees, 359.degrees))
        assertEquals(360.degrees, calc.calculateNewHeadingDegrees(270.degrees, 0.degrees))
        assertEquals(360.degrees, calc.calculateNewHeadingDegrees(359.degrees, 0.degrees))
        assertEquals(370.degrees, calc.calculateNewHeadingDegrees(360.degrees, 10.degrees))
        assertEquals(710.degrees, calc.calculateNewHeadingDegrees(720.degrees, 350.degrees))
        assertEquals(370.degrees, calc.calculateNewHeadingDegrees(720.degrees, 10.degrees))
        assertEquals(90.degrees, calc.calculateNewHeadingDegrees(180.degrees, 90.degrees))
        assertEquals(360.degrees, calc.calculateNewHeadingDegrees(270.degrees, 0.degrees))
        assertEquals(450.degrees, calc.calculateNewHeadingDegrees(540.degrees, 90.degrees))
        assertEquals(1.degrees, calc.calculateNewHeadingDegrees(180.degrees, 1.degrees))
        assertEquals(359.degrees, calc.calculateNewHeadingDegrees(180.degrees, 359.degrees))
        assertEquals(270.degrees, calc.calculateNewHeadingDegrees(270.degrees, 270.degrees))
        assertEquals(361.degrees, calc.calculateNewHeadingDegrees(720.degrees, 1.degrees))
        assertEquals(361.degrees, calc.calculateNewHeadingDegrees(720.degrees, 721.degrees))
        assertEquals(359.degrees, calc.calculateNewHeadingDegrees(0.degrees, (-1).degrees))
    }

    @Test
    fun `test min angle less than zero`() {
        val calc = TurretCalculator(-720.degrees, 720.degrees)
        assertEquals(0.degrees, calc.calculateNewHeadingDegrees(0.degrees, 0.degrees))
        assertEquals(-720.degrees, calc.calculateNewHeadingDegrees(-720.degrees, -720.degrees))
        assertEquals(-720.degrees, calc.calculateNewHeadingDegrees(-720.degrees, 0.degrees))
        assertEquals(-720.degrees, calc.calculateNewHeadingDegrees(-720.degrees, 360.degrees))
        assertEquals(-540.degrees, calc.calculateNewHeadingDegrees(-720.degrees, 180.degrees))
        assertEquals(-1.degrees, calc.calculateNewHeadingDegrees(-1.degrees, 359.degrees))
        assertEquals(-359.degrees, calc.calculateNewHeadingDegrees(-380.degrees, 1.degrees))
    }
}
