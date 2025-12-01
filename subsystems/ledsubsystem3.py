from commands2 import Subsystem
from constants import LEDConstants
from wpilib import PWM


class LEDs(Subsystem):
    def __init__(self):
        super().__init__()

        self.state_constants = {
            "default": 0,
            "scoring_element_held": 1005,
            "green_flashing" : 1105,
            "red_flashing": 1205,
            "white_flashing": 1305,
            "blue_flashing": 1405,
            "yellow_flashing": 1500,
            "_____": 1595,
            "_________": 1695,
            "red_yellow_chaser": 1795,
            "blue_yellow_chaser": 1895,
            "rainbow": 1995,
        }

        self.pwm = PWM(LEDConstants.port)

        self.state = "default"
        self.last_state = "default"
        self.pwm.setPulseTime(self.state_constants[self.state])

    def set_state(self, target_state: str) -> None:
        """Set the current state of the subsystem."""
        self.state = target_state

    def get_state(self) -> str:
        return self.state

    def periodic(self) -> None:
        if self.last_state != self.state:
            self.pwm.setPulseTime(self.state_constants[self.state])
            self.last_state = self.state
