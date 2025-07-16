from commands2 import Command
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.armsubsystem import ArmSubsystem
from subsystems.ledsubsystem import LEDs
from wpilib import DriverStation

class Shoot(Command):

    def __init__(self, drivetrain: CommandSwerveDrivetrain, arm: ArmSubsystem, leds: LEDs):
        super().__init__()
        self.drive = drivetrain
        self.arm = arm
        self.leds = leds

        self.shoot_buffer = [False] * 25
        self.shot_taken = False

        self.addRequirements(arm)
        self.addRequirements(leds)

    def initialize(self):
        self.shoot_buffer = [False] * 25
        self.shot_taken = False
        if self.check_to_use():
            self.arm.set_state("reverse_shoot")
        else:
            self.arm.set_state("shoot")


    def execute(self):
        if self.arm.get_at_target():
            self.shoot_buffer[0] = True
        self.shoot_buffer = self.shoot_buffer[-1:] + self.shoot_buffer[:-1]

        if all(self.shoot_buffer) and not self.shot_taken:
            self.arm.intake.setVoltage(12)
            self.leds.set_state("shoot")
            self.shot_taken = True

    def isFinished(self) -> bool:
        return self.leds.shoot_notification

    def end(self, interrupted: bool):
        self.leds.reset_shoot()
        self.leds.set_state("default")
        self.arm.set_state("stow")

    def check_to_use(self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if self.drive.get_pose().x >= 13.067:
                return -90 < self.drive.get_pose().rotation().degrees() < 90
            else:
                return not -90 < self.drive.get_pose().rotation().degrees() < 90
        else:
            if self.drive.get_pose().x <= 4.484:
                return not -90 < self.drive.get_pose().rotation().degrees() < 90
            else:
                return -90 < self.drive.get_pose().rotation().degrees() < 90