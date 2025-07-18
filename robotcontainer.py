from commands2.cmd import run, runOnce, runEnd
import wpilib.simulation
from commands2 import Command, button, SequentialCommandGroup, ParallelCommandGroup, ParallelRaceGroup, sysid, \
    InterruptionBehavior, ParallelDeadlineGroup, WaitCommand, ConditionalCommand

from constants import OIConstants
from subsystems.armsubsystem import ArmSubsystem
from subsystems.ledsubsystem import LEDs
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.command_swerve_drivetrain import ResetCLT, SetRotation, SetCLTTarget
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, Timer, Alert, Joystick, \
    XboxController
from wpimath.filter import SlewRateLimiter
from pathplannerlib.auto import NamedCommands, AutoBuilder

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve, SignalLogger
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from math import pi, pow, copysign, atan2

from commands.baseline import Baseline
from commands.check_drivetrain import CheckDrivetrain
from commands.alignment_leds import AlignmentLEDs
from commands.profiled_target import ProfiledTarget
from commands.auto_alignment_multi_feedback import AutoAlignmentMultiFeedback
from commands.wheel_radius_calculator import WheelRadiusCalculator
from commands.start_auto_timer import StartAutoTimer
from commands.stop_auto_timer import StopAutoTimer
from commands.pathfollowing_endpoint import PathfollowingEndpointClose
from commands.shoot import Shoot

# Controller layout: https://www.padcrafter.com/?templates=CubeToot%27r+Driver+Controller&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&rightTrigger=%28HOLD%29+Slow+Mode&leftTrigger=%28HOLD%29+Brake&leftBumper=%28HOLD%29+Intake&rightBumper=Shoot&dpadUp=Flick+Heading&dpadRight=Flick+Heading&dpadLeft=Flick+Heading&dpadDown=Flick+Heading&yButton=Reset+Pose+at+Alpha+Point&startButton=Strobe+Lights&leftStickClick=Translate&rightStick=Rotate


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Start master timer. ------------------------------------------------------------------------------------------
        self.timer = Timer()
        self.timer.start()

        # Configure button to enable robot logging.
        self.logging_button = SmartDashboard.putBoolean("Logging Enabled?", False)

        # Disable automatic CTR logging
        SignalLogger.enable_auto_logging(False)

        # Configure system logging. ------------------------------------------------------------------------------------
        self.alert_logging_enabled = Alert("Robot Logging is Enabled", Alert.AlertType.kWarning)
        if wpilib.RobotBase.isReal():
            if SmartDashboard.getBoolean("Logging Enabled?", False) is True:
                DataLogManager.start()
                DriverStation.startDataLog(DataLogManager.getLog(), True)
                SignalLogger.start()
                self.alert_logging_enabled.set(True)
            else:
                SignalLogger.stop()
        else:
            SignalLogger.stop()

        # Startup subsystems. ------------------------------------------------------------------------------------------
        self.leds = LEDs(self.timer)
        self.util = UtilSubsystem()
        self.arm = ArmSubsystem()

        # Setup driver & operator controllers. -------------------------------------------------------------------------
        self.driver_controller = button.CommandXboxController(OIConstants.kDriverControllerPort)
        self.operator_controller = button.CommandXboxController(OIConstants.kOperatorControllerPort)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.test_bindings = False

        # Configure drivetrain settings. -------------------------------------------------------------------------------
        self._max_speed = TunerConstants.speed_at_12_volts  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        self._drive = (
            swerve.requests.FieldCentric()  # I want field-centric
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  # Add a 10% deadband
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._hold_heading = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self._hold_heading.heading_controller.setPID(5, 0, 0)
        self._hold_heading.heading_controller.enableContinuousInput(0, -2 * pi)
        self._hold_heading.heading_controller.setTolerance(0.1)  # 0.1

        # Register commands for PathPlanner. ---------------------------------------------------------------------------
        self.registerCommands()

        SmartDashboard.putBoolean("Misalignment Indicator Active?", False)
        SmartDashboard.putNumber("Misalignment Angle", 0)

        # Setup for all event-trigger commands. ------------------------------------------------------------------------
        # self.configureTriggersSmartDash()
        self.configure_test_bindings()
        self.configure_triggers()

        # Setup autonomous selector on the dashboard. ------------------------------------------------------------------
        self.m_chooser = AutoBuilder.buildAutoChooser("DoNothing")
        SmartDashboard.putData("Auto Select", self.m_chooser)

        self.drive_filter_x = SlewRateLimiter(3, -3, 0)
        self.drive_filter_y = SlewRateLimiter(3, -3, 0)

    def configure_triggers(self) -> None:
        # NON CLT DRIVING
        # self.drivetrain.setDefaultCommand(  # Drivetrain will execute this command periodically
        #     self.drivetrain.apply_request(
        #         lambda: (
        #             self._drive.with_velocity_x(
        #                 -copysign(pow(self.drive_filter_x.calculate(self.driver_controller.getLeftY()), 1),
        #                           self.drive_filter_x.calculate(self.driver_controller.getLeftY()))
        #                 * self._max_speed * self.elevator_and_arm.get_accel_limit())
        #             .with_velocity_y(-copysign(pow(self.drive_filter_y.calculate(self.driver_controller.getLeftX()), 1),
        #                                        self.drive_filter_y.calculate(self.driver_controller.getLeftX()))
        #                              * self._max_speed * self.elevator_and_arm.get_accel_limit())
        #             .with_rotational_rate(-copysign(pow(self.driver_controller.getRightX(), 1),
        #                                             self.driver_controller.getRightX())
        #                                   * self._max_angular_rate * self.elevator_and_arm.get_accel_limit())
        #         )
        #     )
        # )

        # Slow mode NON CLT DRIVING
        # (self.driver_controller.rightTrigger().and_(lambda: not self.driver_controller.x().getAsBoolean())
        # .and_(lambda: not self.driver_controller.b().getAsBoolean()).whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: (
        #             self._drive.with_velocity_x(
        #                 -self.drive_filter_x.calculate(self.driver_controller.getLeftY())
        #                 * self._max_speed * 0.4)
        #             .with_velocity_y(
        #                 -self.drive_filter_y.calculate(self.driver_controller.getLeftX())
        #                 * self._max_speed * 0.4)
        #             .with_rotational_rate(
        #                 -self.driver_controller.getRightX()
        #                 * self._max_angular_rate * 0.4)
        #         )
        #     )
        # ))

        self.drivetrain.setDefaultCommand(
                self.drivetrain.apply_request(
                    lambda: (
                        self.drivetrain.drive_clt(
                            self.drive_filter_y.calculate(self.driver_controller.getLeftY()) * self._max_speed * -1,
                            self.drive_filter_x.calculate(self.driver_controller.getLeftX()) * self._max_speed * -1,
                            self.driver_controller.getRightX() * -1
                        )
                    )
                )
            )

        self.driver_controller.rightTrigger().and_(lambda: not self.test_bindings).whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self.drivetrain.drive_clt(
                        self.drive_filter_y.calculate(
                            self.driver_controller.getLeftY()) * self._max_speed * -1 * 0.2,
                        self.drive_filter_x.calculate(
                            self.driver_controller.getLeftX()) * self._max_speed * -1 * 0.2,
                        self.driver_controller.getRightX() * -1 * 0.2
                    )
                )
            )
        )

        # POV Snap mode
        self.driver_controller.povUp().onTrue(
            SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(180))
        )
        self.driver_controller.povUpLeft().onTrue(
            SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(225))
        )
        self.driver_controller.povLeft().onTrue(
            SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(270))
        )
        self.driver_controller.povDownLeft().onTrue(
            SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(315))
        )
        self.driver_controller.povRight().onTrue(
            SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(90))
        )
        self.driver_controller.povDownRight().onTrue(
            SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(45))
        )
        self.driver_controller.povUpRight().onTrue(
            SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(135))
        )
        self.driver_controller.povDown().onTrue(
            SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(0))
        )

        # Reset pose.
        self.driver_controller.y().and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.drivetrain.reset_odometry(), self.drivetrain).ignoringDisable(True),
                ResetCLT(self.drivetrain).ignoringDisable(True)
            )
        )

        # Auto Alignment
        self.driver_controller.b().and_(lambda: not self.test_bindings).whileTrue(
            AutoAlignmentMultiFeedback(self.drivetrain, self.util, self.driver_controller, False)
        ).onFalse(
            ResetCLT(self.drivetrain)
        ).onTrue(
            runOnce(lambda: self.arm.set_state("shoot"), self.arm)
        )
        self.driver_controller.x().and_(lambda: not self.test_bindings).whileTrue(
            AutoAlignmentMultiFeedback(self.drivetrain, self.util, self.driver_controller, True)
        ).onFalse(
            ResetCLT(self.drivetrain)
        ).onTrue(
            runOnce(lambda: self.arm.set_state("reverse_shoot"), self.arm)
        )

        # Human player LEDs
        self.driver_controller.start().and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.leds.set_flash_color_rate(15), self.leds),
                runOnce(lambda: self.leds.set_flash_color_color([255, 255, 255]), self.leds),
                runOnce(lambda: self.leds.set_state("flash_color"), self.leds)
            ).ignoringDisable(True)
        ).onFalse(
            runOnce(lambda: self.leds.set_state("default"), self.leds).ignoringDisable(True)
        )

        # Reset all pose based on vision data.
        self.driver_controller.back().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.drivetrain.select_best_vision_pose((0.00001, 0.00001, 0.00001)))
        )

        # Drivetrain brake mode.
        self.driver_controller.leftTrigger().and_(lambda: not self.test_bindings).whileTrue(
            self.drivetrain.apply_request(lambda: self._brake)
        )

        # Intake with LB.
        self.driver_controller.leftBumper().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.arm.set_state("intake"), self.arm)
        ).onFalse(
            runOnce(lambda: self.arm.set_state("stow"), self.arm)
        )

        # Shoot
        self.driver_controller.rightBumper().and_(lambda: not self.test_bindings).onTrue(
            Shoot(self.drivetrain, self.arm, self.leds)
        )

        # Configuration for telemetry.
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    # def configureTriggersSmartDash(self) -> None:
    # Activate autonomous misalignment lights.
    # button.Trigger(lambda: SmartDashboard.getBoolean("Misalignment Indicator Active?", False)).whileTrue(
    #     AutoAlignmentLEDs(self.drivetrain, self.leds, self.m_auto_start_location)
    #     .ignoringDisable(True)
    # )

    # button.Trigger(lambda: SmartDashboard.getBoolean("Logging Enabled?", False)).onTrue(
    #     SequentialCommandGroup(
    #         runOnce(lambda: DataLogManager.start()),
    #         runOnce(lambda: DriverStation.startDataLog(DataLogManager.getLog(), True)),
    #         runOnce(lambda: SignalLogger.start()),
    #         runOnce(lambda: self.alert_logging_enabled.set(True))
    #     )
    # ).onFalse(
    #     SequentialCommandGroup(
    #         runOnce(lambda: DataLogManager.stop()),
    #         runOnce(lambda: SignalLogger.stop()),
    #         runOnce(lambda: self.alert_logging_enabled.set(False))
    #     )
    # )
    def get_autonomous_command(self) -> Command:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        return self.m_chooser.getSelected()

    def configure_test_bindings(self) -> None:
        self.configure_sys_id()

        # Point all modules in a direction
        self.driver_controller.start().and_(lambda: self.test_bindings).whileTrue(self.drivetrain.apply_request(
            lambda: self._point.with_module_direction(
                Rotation2d(-1 * self.driver_controller.getLeftY()
                           - 1 * self.driver_controller.getLeftX()))))

        self.driver_controller.back().and_(lambda: self.test_bindings).onTrue(
            WheelRadiusCalculator(self.drivetrain, self.timer)
        )

    def configure_sys_id(self) -> None:
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kReverse)))

    def enable_test_bindings(self, enabled: bool) -> None:
        self.test_bindings = enabled

    def check_endpoint_closed(self) -> bool:
        return self.drivetrain.endpoint[0] - 0.02 < self.drivetrain.get_pose().x < self.drivetrain.endpoint[
            0] + 0.02 and self.drivetrain.endpoint[
            1] - 0.02 < self.drivetrain.get_pose().y < self.drivetrain.endpoint[1] + 0.02

    def registerCommands(self):
        NamedCommands.registerCommand("rainbow_leds", runOnce(lambda: self.leds.set_state("rainbow"),
                                                              self.leds))
        NamedCommands.registerCommand("flash_green",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([255, 0, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_red",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([0, 255, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_blue",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([0, 0, 255]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_purple",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([50, 149, 168]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("flash_yellow",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.leds.set_flash_color_color([255, 255, 0]),
                                                  self.leds),
                                          runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
                                          runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        NamedCommands.registerCommand("default_leds", runOnce(lambda: self.leds.set_state("default"),
                                                              self.leds))
        NamedCommands.registerCommand("baseline", Baseline(self.drivetrain, self.timer))
        NamedCommands.registerCommand("check_drivetrain", CheckDrivetrain(self.drivetrain, self.timer))
        NamedCommands.registerCommand("override_heading_goal",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.drivetrain.set_lookahead(True)),
                                          runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("goal"))
                                        )
                                      )
        NamedCommands.registerCommand("override_heading_gp",
                                      runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("gp")))
        NamedCommands.registerCommand("disable_override_heading",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.drivetrain.set_lookahead(False)),
                                          runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("none"))
                                      ))
        NamedCommands.registerCommand("start_timer", StartAutoTimer(self.util, self.timer))
        NamedCommands.registerCommand("stop_timer", StopAutoTimer(self.util, self.timer))
        NamedCommands.registerCommand("close_j",
                                          SequentialCommandGroup(
                                              PathfollowingEndpointClose(self.drivetrain, [7.133, 5.223, -120]),
                                              self.drivetrain.apply_request(self.drivetrain.saved_request)
                                          ).onlyWhile(lambda: self.check_endpoint_closed())
                                      )
        NamedCommands.registerCommand("reset_CLT", ResetCLT(self.drivetrain))
