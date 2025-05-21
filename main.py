import RPi.GPIO as GPIO
import time
import serial
from enum import Enum, auto

# Serial command constants
LID_CLOSED_CMD = b"1000"
LID_OPEN_CMD = b"1001"

BRIGHTNESS_OFF_CMD = b"9999"
BRIGHTNESS_LOW_CMD = b"10"
BRIGHTNESS_HIGH_CMD = b"100"

HEATER_OFF_CMD = b"2000"
HEATER_LOW_CMD = b"2050"
HEATER_HIGH_CMD = b"2100"
HEATER_MAX_CMD = b"2150"


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


def dispatchToWandererCover(lid_status, brightness, heater):
    global ser

    # Try to connect if not connected
    if ser is None:
        if connect_serial():
            print("Successfully reconnected to serial port")
        else:
            print("Serial communication not available, will retry next time")
            return

    try:
        # Send lid status command
        if lid_status == LidStatus.CLOSED:
            ser.write(LID_CLOSED_CMD)
            print(f"Sent lid command: {LID_CLOSED_CMD}")
        elif lid_status == LidStatus.OPEN:
            ser.write(LID_OPEN_CMD)
            print(f"Sent lid command: {LID_OPEN_CMD}")

        # Send brightness command
        if brightness == Brightness.OFF:
            ser.write(BRIGHTNESS_OFF_CMD)
            print(f"Sent brightness command: {BRIGHTNESS_OFF_CMD}")
        elif brightness == Brightness.LOW:
            ser.write(BRIGHTNESS_LOW_CMD)
            print(f"Sent brightness command: {BRIGHTNESS_LOW_CMD}")
        elif brightness == Brightness.HIGH:
            ser.write(BRIGHTNESS_HIGH_CMD)
            print(f"Sent brightness command: {BRIGHTNESS_HIGH_CMD}")

        # Send heater command
        if heater == HeaterPower.OFF:
            ser.write(HEATER_OFF_CMD)
            print(f"Sent heater command: {HEATER_OFF_CMD}")
        elif heater == HeaterPower.LOW:
            ser.write(HEATER_LOW_CMD)
            print(f"Sent heater command: {HEATER_LOW_CMD}")
        elif heater == HeaterPower.HIGH:
            ser.write(HEATER_HIGH_CMD)
            print(f"Sent heater command: {HEATER_HIGH_CMD}")

    except Exception as e:
        print(f"Error communicating with device: {e}")
        # Reset the connection to force a reconnect next time
        try:
            if ser:
                ser.close()
        except Exception:
            pass
        ser = None


# Initialize GPIO
GPIO.setmode(GPIO.BCM)

# Create the switches with their specific types
switches = [
    LidSwitch("Lid", 17, 27),
    BrightnessSwitch("Brightness", 22, 23),
    HeaterSwitch("Heater", 24, 25),
]


# Initialize serial connection
ser = None
serial_port = "/dev/ttyUSB0"
baud_rate = 19200
max_retries = 5
retry_delay = 2  # Seconds between retry attempts


def connect_serial():
    """Attempt to connect to the serial port."""
    global ser
    if ser is not None:
        return True

    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Successfully connected to {serial_port}")
        return True
    except Exception as e:
        print(f"Warning: Could not open serial port {serial_port}: {e}")
        return False


# Try to connect initially
connect_serial()


try:
    print("Reading switch positions. Press CTRL+C to exit.")
    print("\nStarting monitoring loop...\n")

    last_connection_attempt = 0

    while True:
        current_time = time.time()

        # Try to reconnect periodically if no connection
        if ser is None and (current_time - last_connection_attempt) > retry_delay:
            print("Attempting to reconnect to serial port...")
            connect_serial()
            last_connection_attempt = current_time

        # Update all switch positions
        for switch in switches:
            switch.read()
            print(switch)

        # Access specific states if needed
        lid_status = switches[0].get_state_value()
        brightness = switches[1].get_state_value()
        heater = switches[2].get_state_value()

        # Dispatch the states to serial COM
        dispatchToWandererCover(lid_status, brightness, heater)

        # Read any incoming serial data
        if ser and ser.in_waiting:
            try:
                received_data = ser.read(ser.in_waiting)
                if received_data:
                    print(f"Received from device: {received_data}")
            except Exception as e:
                print(f"Error reading from serial: {e}")
                # Reset connection
                try:
                    if ser:
                        ser.close()
                except Exception:
                    pass
                ser = None

        print("---")
        time.sleep(2)
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    # Clean up GPIO
    GPIO.cleanup()
    # Close serial connection if open
    if ser:
        ser.close()
        print("Serial connection closed.")
