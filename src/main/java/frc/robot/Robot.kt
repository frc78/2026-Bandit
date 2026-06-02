package frc.robot

// import frc.robot.auto.RBRBOC
import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.net.WebServer
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import frc.robot.auto.*
import frc.robot.generated.Telemetry
import frc.robot.lib.PeriodTimer
import frc.robot.lib.hz
import frc.robot.lib.radians
import frc.robot.subsystems.Climber
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.LED
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.SwerveDrivetrain
import frc.robot.vision.VisionSim
import kotlin.math.PI
import kotlin.math.cos
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher

/**
 * The functions in this object (which basically functions as a singleton class) are called
 * automatically corresponding to each mode, as described in the TimedRobot documentation. This is
 * written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also
 * update the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when
 * renaming the object or package, it will get changed everywhere.)
 */
object Robot : LoggedRobot() {
    init {
        // Kotlin initializer block, which effectually serves as the constructor code.
        // https://kotlinlang.org/docs/classes.html#constructors
        // This work can also be done in the inherited `robotInit()` method. But as of the 2025
        // season the
        // `robotInit` method's Javadoc encourages using the constructor and the official templates
        // moved initialization code out `robotInit` and into the constructor. We follow suit in
        // Kotlin.

        // Report the use of the Kotlin Language for "FRC Usage Report" statistics.
        // Please retain this line so that Kotlin's growing use by teams is seen by FRC/WPI.
        HAL.report(
            tResourceType.kResourceType_Language,
            tInstances.kLanguage_Kotlin,
            0,
            WPILibVersion.Version,
        )
        Logger.addDataReceiver(NT4Publisher())
        Logger.start()
        if (isReal()) {
            // DataLog will automatically log all NT changes. AKit logs to NT, DataLog logs NT to
            // file
            DataLogManager.start()
            SignalLogger.setPath("/U/ctre-logs")
            SignalLogger.start()
        }
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())
        SwerveDrivetrain.registerTelemetry(Telemetry::telemeterize)
        Notifier(Indexer::countFuel).startPeriodic(100.hz)
        //        Notifier(Shooter::measureShotTime).startPeriodic(100.hz)
        Notifier {
                if (DriverStation.isEnabled()) {
                    DataLogManager.getLog().resume()
                }
            }
            .startPeriodic(1.hz)
        WebServer.start(5800, Filesystem.getDeployDirectory().path)
    }

    override fun robotPeriodic() {
        SwerveDrivetrain.periodic()
        SwerveDrivetrain.updateLimelights()
        LED.stateMachine()
        Intake.periodic()
        Indexer.periodic()

        // Execute actions for active states in all subsystems except drivetrain
        Shooter.stateActions()
        Intake.stateActions()
        Climber.stateActions()
        Indexer.stateTransitions()
        Indexer.stateActions()

        val turretPosition =
            Pose3d(-.127, .152, .464, Rotation3d(0.0, 0.0, Shooter.turretAngle.radians))
        val hoodPosition =
            turretPosition.transformBy(
                Transform3d(.102, 0.0, .090, Rotation3d(0.0, Shooter.hoodAngle.radians, 0.0))
            )
        val pivotPosition = Pose3d(.254, 0.0, .222, Rotation3d(0.0, Intake.pivotAngle.radians, 0.0))
        val hopperPosition =
            Pose3d(
                .3048 * cos(Intake.pivotAngle.radians + 4 * PI / 3).coerceAtLeast(0.0),
                0.0,
                0.0,
                Rotation3d.kZero,
            )
        val climberPosition = Pose3d(0.0, 0.0, Climber.climberHeight, Rotation3d.kZero)
        Logger.recordOutput(
            "ComponentPoses",
            turretPosition,
            hoodPosition,
            pivotPosition,
            hopperPosition,
            climberPosition,
        )
    }

    override fun autonomousInit() {
        autoSelector.selected?.init()
    }

    // val auto = DC

    private val autoSelector =
        SendableChooser<Auto>().apply {
            setDefaultOption("RB-RB-O-C", RBRBOC)
            addOption("LB-LB-D-C", LBLBDC)
            addOption("RB-LB-D-C", RBLBDC)
            addOption("LB-RB-O-C", LBRBOC)
            addOption("LB-LB", LBLB)
            addOption("O-C", OC)
            addOption("D-C", DC)
            addOption("RB-RB", RBRB)
            addOption("OD", OD)
            addOption("LBLBD_Shallow", LBLBD_Shallow)
            addOption("LB-LB-D-Square-Bump", LBLBD_SquareBump)
            SmartDashboard.putData("Auto Mode", this)
        }

    override fun autonomousPeriodic() {
        autoSelector.selected?.run()
    }

    override fun teleopInit() {
        PeriodTimer.startPeriodTimer()
    }

    override fun teleopPeriodic() {
        // Update controller inputs and change states accordingly
        // We do this only in teleop so auto modes can set states manually without conflict
        OI.periodic()

        // Update state machine transitions
        SwerveDrivetrain.stateTransitions()
        Shooter.stateTransitions()
        Intake.stateTransitions()
        Climber.stateTransitions()

        // Auto uses driveToPose() separately of state-based actions,
        //  so we only need to do these for the drivetrain in teleop
        SwerveDrivetrain.stateActions()

        PeriodTimer.refresh()
    }

    private val field2d = Field2d()

    override fun disabledInit() {
        SmartDashboard.putData(field2d)
    }

    override fun disabledPeriodic() {
        field2d.robotPose = SwerveDrivetrain.state.Pose
    }

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {
        Logger.recordOutput("ZeroedPoses", Pose3d(), Pose3d(), Pose3d(), Pose3d(), Pose3d())
        VisionSim.setupSimulation()
        Shooter.simulationInit()
    }

    override fun simulationPeriodic() {
        // VisionSim.update()
        Shooter.simulationPeriodic()
        Intake.simulationPeriodic()
        Climber.simulationPeriodic()
    }
}
