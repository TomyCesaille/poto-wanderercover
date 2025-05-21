import RPi.GPIO as GPIO
import time
import serial
from enum import Enum, auto
from collections import deque

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

# State tracking variables
last_lid_status = None
last_brightness = None
last_heater = None
command_queue = deque()
waiting_for_response = False
last_command_time = 0
command_timeout = 3  # seconds
first_message_received = (
    False  # Flag to track if we've received the first message from the device
)


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


def check_for_state_changes(lid_status, brightness, heater):
    """Check for changes in state and add commands to the queue if needed"""
    global last_lid_status, last_brightness, last_heater, command_queue

    # Check if this is the first run or if there are changes in lid status
    if last_lid_status is None or last_lid_status != lid_status:
        if lid_status == LidStatus.CLOSED:
            command_queue.append((LID_CLOSED_CMD, "Lid: CLOSED"))
        elif lid_status == LidStatus.OPEN:
            command_queue.append((LID_OPEN_CMD, "Lid: OPEN"))
        last_lid_status = lid_status

    # Check for changes in brightness
    if last_brightness is None or last_brightness != brightness:
        if brightness == Brightness.OFF:
            command_queue.append((BRIGHTNESS_OFF_CMD, "Brightness: OFF"))
        elif brightness == Brightness.LOW:
            command_queue.append((BRIGHTNESS_LOW_CMD, "Brightness: LOW"))
        elif brightness == Brightness.HIGH:
            command_queue.append((BRIGHTNESS_HIGH_CMD, "Brightness: HIGH"))
        last_brightness = brightness

    # Check for changes in heater
    if last_heater is None or last_heater != heater:
        if heater == HeaterPower.OFF:
            command_queue.append((HEATER_OFF_CMD, "Heater: OFF"))
        elif heater == HeaterPower.LOW:
            command_queue.append((HEATER_LOW_CMD, "Heater: LOW"))
        elif heater == HeaterPower.HIGH:
            command_queue.append((HEATER_HIGH_CMD, "Heater: HIGH"))
        last_heater = heater


def process_command_queue():
    """Process the next command in the queue if possible"""
    global ser, command_queue, waiting_for_response, last_command_time, first_message_received

    # Don't send any commands until we've received the first message from the device
    if not first_message_received:
        return

    # If we're waiting for a response but it's been too long, timeout
    current_time = time.time()
    if waiting_for_response and (current_time - last_command_time) > command_timeout:
        print(f"Command timed out after {command_timeout} seconds, continuing...")
        waiting_for_response = False

    # If we're not currently waiting for a response and have commands to send
    if not waiting_for_response and command_queue and ser is not None:
        try:
            command, description = command_queue.popleft()
            ser.write(command)
            print(f"Sent command: {command} ({description})")
            waiting_for_response = True
            last_command_time = current_time
        except Exception as e:
            print(f"Error sending command: {e}")
            # Reset the connection to force a reconnect next time
            try:
                if ser:
                    ser.close()
            except Exception:
                pass
            ser = None


def dispatchToWandererCover(lid_status, brightness, heater):
    """Add any changed states to the command queue"""
    global ser

    # Try to connect if not connected
    if ser is None:
        if connect_serial():
            print("Successfully reconnected to serial port")
            # Force all states to be resent after reconnection
            global last_lid_status, last_brightness, last_heater
            last_lid_status = None
            last_brightness = None
            last_heater = None
        else:
            print("Serial communication not available, will retry next time")
            return

    # Check for state changes and add to queue
    check_for_state_changes(lid_status, brightness, heater)


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
    print("Waiting for first message from device before sending any commands...")

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

        # Check for state changes and add to command queue
        dispatchToWandererCover(lid_status, brightness, heater)

        # Process next command in queue if we're not waiting for a response
        process_command_queue()

        # Read any incoming serial data
        if ser and ser.in_waiting:
            try:
                received_data = ser.read(ser.in_waiting)
                if received_data:
                    print(f"Received from device: {received_data}")
                    if not first_message_received:
                        first_message_received = True
                        print(
                            "First message received from device, now ready to send commands"
                        )
                    waiting_for_response = False
                    process_command_queue()
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
        time.sleep(1)
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    # Clean up GPIO
    GPIO.cleanup()
    # Close serial connection if open
    if ser:
        ser.close()
        print("Serial connection closed.")
