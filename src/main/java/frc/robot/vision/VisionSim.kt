package frc.robot.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.Notifier
import frc.robot.lib.degrees
import frc.robot.lib.inches
import frc.robot.subsystems.SwerveDrivetrain
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim

object VisionSim {
    private val cams: List<Camera> =
        listOf(
            Camera(
                "Left",
                Transform3d(
                    (-5.727).inches,
                    13.231.inches,
                    15.591.inches,
                    Rotation3d(0.degrees, (-20).degrees, 90.degrees),
                ),
            ),
            Camera(
                "Rear",
                Transform3d(
                    (-12.728).inches,
                    6.749.inches,
                    15.602.inches,
                    Rotation3d(0.degrees, (-20).degrees, 180.degrees),
                ),
            ),
            Camera(
                "Right",
                Transform3d(
                    -8.749.inches,
                    -.228.inches,
                    15.602.inches,
                    Rotation3d(0.degrees, (-20).degrees, 270.degrees),
                ),
            ),
        )

    fun update() {
        cams.forEach(Camera::updatePoseEstimate)
    }

    /** Initializes simulated cameras and runs an update thread at 50 Hz */
    fun setupSimulation() {
        val visionSim = VisionSystemSim("sim")
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded))

        val camProps =
            SimCameraProperties().apply {
                setCalibration(1200, 720, Rotation2d.fromDegrees(70.0))
                fps = 25.0
            }

        cams.forEach {
            visionSim.addCamera(
                PhotonCameraSim(it.cam, camProps).apply {
                    enableRawStream(true)
                    enableDrawWireframe(true)
                },
                it.transform,
            )
        }

        Notifier { visionSim.update(SwerveDrivetrain.state.Pose) }.startPeriodic(0.020)
    }
}
