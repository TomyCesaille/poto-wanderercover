import RPi.GPIO as GPIO
import time
from enum import Enum, auto


class SwitchPosition(Enum):
    """Enum for the physical positions of a 3-way switch."""

    POSITION1 = 1  # First position (left)
    CENTER = 0  # Center position (off)
    POSITION2 = 2  # Second position (right)
    INVALID = -1  # Invalid state (both pins high)


class LidStatus(Enum):
    """Lid status options (Switch 1)."""

    CLOSED = auto()  # Lid is closed (left or center position)
    OPEN = auto()  # Lid is open (right position)


class Brightness(Enum):
    """Brightness levels (Switch 2)."""

    OFF = auto()  # No lights (left position)
    LOW = auto()  # Low brightness (center position)
    HIGH = auto()  # High brightness (right position)


class HeaterPower(Enum):
    """Heater power levels (Switch 3)."""

    OFF = auto()  # No heat (left position)
    LOW = auto()  # Low heat (center position)
    HIGH = auto()  # High heat (right position)


class Switch:
    """
    Represents a 3-position switch connected to two GPIO pins.
    """

    def __init__(self, name, pin_a, pin_b, switch_type=None):
        """
        Initialize a new switch.

        Args:
            name: A descriptive name for the switch.
            pin_a: The GPIO pin number for position 1.
            pin_b: The GPIO pin number for position 2.
            switch_type: The type of switch (LidSwitch, BrightnessSwitch, HeaterSwitch)
        """
        self.name = name
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = SwitchPosition.CENTER
        self.switch_type = switch_type
        # Setup the pins
        GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def read(self):
        """
        Read the current position of the switch and update the position.

        Returns:
            The current position as a SwitchPosition enum.
        """
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)

        if a and not b:
            self.position = SwitchPosition.POSITION1
        elif not a and b:
            self.position = SwitchPosition.POSITION2
        elif not a and not b:
            self.position = SwitchPosition.CENTER
        else:
            self.position = SwitchPosition.INVALID
        return self.position

    def get_position_string(self):
        """Get a human-readable string for the current position."""
        positions = {
            SwitchPosition.POSITION1: "Position 1 (Left)",
            SwitchPosition.POSITION2: "Position 2 (Right)",
            SwitchPosition.CENTER: "Center (OFF)",
            SwitchPosition.INVALID: "Invalid (both HIGH)",
        }
        return positions.get(self.position, "Unknown")

    def get_state_value(self):
        """Get the functional state value. To be implemented by subclasses."""
        return None

    def __str__(self):
        """String representation of the switch."""
        state = self.get_state_value()
        state_str = f" → {state.name}" if state else ""
        return f"{self.name}: {self.get_position_string()}{state_str}"


class LidSwitch(Switch):
    """Switch controlling the lid status."""

    def get_state_value(self):
        """Get the lid status based on switch position."""
        if self.position == SwitchPosition.POSITION2:
            return LidStatus.OPEN
        else:
            # Both position 1 and center mean closed
            return LidStatus.CLOSED


class BrightnessSwitch(Switch):
    """Switch controlling the brightness level."""

    def get_state_value(self):
        """Get the brightness level based on switch position."""
        if self.position == SwitchPosition.POSITION1:
            return Brightness.OFF
        elif self.position == SwitchPosition.CENTER:
            return Brightness.LOW
        elif self.position == SwitchPosition.POSITION2:
            return Brightness.HIGH
        return None


class HeaterSwitch(Switch):
    """Switch controlling the heater power."""

    def get_state_value(self):
        """Get the heater power level based on switch position."""
        if self.position == SwitchPosition.POSITION1:
            return HeaterPower.OFF
        elif self.position == SwitchPosition.CENTER:
            return HeaterPower.LOW
        elif self.position == SwitchPosition.POSITION2:
            return HeaterPower.HIGH
        return None


# Initialize GPIO
GPIO.setmode(GPIO.BCM)

# Create the switches with their specific types
switches = [
    LidSwitch("Lid", 17, 27),
    BrightnessSwitch("Brightness", 22, 23),
    HeaterSwitch("Heater", 24, 25),
]


try:
    print("Reading switch positions. Press CTRL+C to exit.")
    print("Lid positions: CLOSED (left/center), OPEN (right)")
    print("Brightness: OFF (left), LOW (center), HIGH (right)")
    print("Heater: OFF (left), LOW (center), HIGH (right)")
    print("\nStarting monitoring loop...\n")

    while True:
        # Update all switch positions
        for switch in switches:
            switch.read()
            print(switch)

        # Access specific states if needed
        lid_status = switches[0].get_state_value()
        brightness = switches[1].get_state_value()
        heater = switches[2].get_state_value()

        # Example of using the states for control logic
        if lid_status == LidStatus.OPEN:
            print("Warning: Lid is open!")

        if heater == HeaterPower.HIGH and brightness == Brightness.OFF:
            print("Note: Heater is HIGH but lights are OFF")

        print("---")
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    GPIO.cleanup()
