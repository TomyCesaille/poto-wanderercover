from enum import Enum, auto
from typing import Literal
import RPi.GPIO as GPIO
import serial
import time
import traceback

from utils_statuses import (
    print_wanderer_status_message,
    print_switches_state,
)

# Constants
# seconds. Before we can send brightness and heater commands again after closing the lid.
# We need to repush commands as they are ignored by the hardware when the lid is moving/open.
TIME_TO_WAIT_AFTER_CLOSE = 20

# Serial command constants.
LID_CLOSE_CMD = b"1000"
LID_OPEN_CMD = b"1001"

BRIGHTNESS_OFF_CMD = b"9999"
BRIGHTNESS_LOW_CMD = b"10"
BRIGHTNESS_HIGH_CMD = b"100"

HEATER_OFF_CMD = b"2000"
HEATER_LOW_CMD = b"2050"
HEATER_HIGH_CMD = b"2100"
HEATER_MAX_CMD = b"2150"

GPIO.setmode(GPIO.BCM)


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

    def __init__(self, name: str, pin_a: int, pin_b: int, switch_type=None) -> None:
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

    def read(
        self,
    ) -> Literal[
        SwitchPosition.POSITION1,
        SwitchPosition.POSITION2,
        SwitchPosition.CENTER,
        SwitchPosition.INVALID,
    ]:
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

    def __str__(self) -> str:
        state = self.get_state_value()
        state_str = f" → {state.name}" if state else ""
        return f"{self.name}: {self.get_position_string()}{state_str}"


class CoverSwitch(Switch):
    """Switch controlling the open/close position."""

    def get_state_value(self) -> LidStatus:
        if self.position == SwitchPosition.POSITION2:
            return LidStatus.OPEN
        else:
            # Both position 1 and center mean closed
            return LidStatus.CLOSED


class BrightnessSwitch(Switch):
    """Switch controlling the brightness level."""

    def get_state_value(self) -> Brightness:
        if self.position == SwitchPosition.POSITION1:
            return Brightness.OFF
        elif self.position == SwitchPosition.CENTER:
            return Brightness.LOW
        return Brightness.HIGH


class DewHeaterSwitch(Switch):
    """Switch controlling the dew heater power."""

    def get_state_value(self) -> HeaterPower:
        if self.position == SwitchPosition.POSITION1:
            return HeaterPower.OFF
        elif self.position == SwitchPosition.CENTER:
            return HeaterPower.LOW
        return HeaterPower.HIGH


def connect_serial_if_needed(ser: serial.Serial | None) -> serial.Serial | None:
    """Connect to serial if disconnected."""
    if ser is not None:
        return ser

    serial_port = "/dev/ttyUSB0"

    try:
        ser = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=19200,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=5,
        )
        print(f"Reading on port {ser.portstr} at {ser.baudrate} baud")
        return ser
    except Exception:
        print(
            f"Warning: Could not open serial port {serial_port}: {traceback.format_exc()}"
        )
        return None


# Allow the wanderer hardware to initialize.
print("starting in 4 seconds...")
# time.sleep(1)
print("starting in 3 seconds...")
# time.sleep(1)
print("starting in 2 seconds...")
# time.sleep(1)
print("starting in 1 second...")
# time.sleep(1)

print("\nStarting the routine...\n")

switches = [
    CoverSwitch("Lid", 17, 27),
    BrightnessSwitch("Brightness", 22, 23),
    DewHeaterSwitch("Heater", 24, 25),
]

ser = None
# Remember the last switch states to avoid sending duplicate commands.
last_openclose_switch_state = None
last_brightness_switch_state = None
last_dew_heater_switch_state = None
lid_close_command_time = None  # Time when close lid command was sent.

try:
    while True:
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        ser = connect_serial_if_needed(ser)

        if ser is None:
            print(f"[{current_time}] Serial connection not established. Retrying...")
            time.sleep(2)
            continue

        # Read the hardware status from the serial port.
        # Receiving a status message from the WandererCover device means that it is ready to take a command.
        hardware_status_raw = ser.readline()
        if not hardware_status_raw:
            print(f"[{current_time}] No data received from serial port.")
            continue

        print_wanderer_status_message(hardware_status_raw)

        # Read GPIO switch positions.
        for switch in switches:
            switch.read()
        openclose_switch_state = switches[0].get_state_value()
        brightness_switch_state = switches[1].get_state_value()
        dew_heater_switch_state = switches[2].get_state_value()
        print_switches_state(
            openclose_switch_state, brightness_switch_state, dew_heater_switch_state
        )

        # Check if it's time to force retrigger brightness and heater commands.
        if lid_close_command_time is not None:
            time_since_close = time.time() - lid_close_command_time
            time_remaining = TIME_TO_WAIT_AFTER_CLOSE - time_since_close

            print(f"[{current_time}] ⏱️ Time since close: {time_since_close:.1f}s")

            if time_since_close >= TIME_TO_WAIT_AFTER_CLOSE:
                print(
                    f"[{current_time}] ⏱️ {TIME_TO_WAIT_AFTER_CLOSE} seconds since lid close command."
                )

                # Toggle brightness state using correct enum types.
                if last_brightness_switch_state == Brightness.OFF:
                    last_brightness_switch_state = Brightness.LOW
                else:
                    last_brightness_switch_state = Brightness.OFF

                # Toggle heater state using correct enum types.
                if last_dew_heater_switch_state == HeaterPower.OFF:
                    last_dew_heater_switch_state = HeaterPower.LOW
                else:
                    last_dew_heater_switch_state = HeaterPower.OFF

                print(
                    f"[{current_time}] 🔄 Toggled brightness and heater states to force retrigger."
                )

                # Reset the timer so we don't do this again.
                lid_close_command_time = None

        # Send 1 command from this loop, if the switch state has changed.
        command_sent = False
        if openclose_switch_state != last_openclose_switch_state:
            if openclose_switch_state == LidStatus.OPEN:
                print(f"[{current_time}] 👉 Sending command to open lid.")
                ser.write(LID_OPEN_CMD)
                # Reset the lid close timer when opening.
                lid_close_command_time = None
            else:
                print(f"[{current_time}] 👉 Sending command to close lid.")
                ser.write(LID_CLOSE_CMD)
                # Start tracking time since close command.
                lid_close_command_time = time.time()
                print(
                    f"[{current_time}] ⏱️ Started {TIME_TO_WAIT_AFTER_CLOSE}-second timer for brightness/heater reset."
                )
            last_openclose_switch_state = openclose_switch_state
            command_sent = True

        if not command_sent and brightness_switch_state != last_brightness_switch_state:
            if brightness_switch_state == Brightness.OFF:
                print(f"[{current_time}] 👉 Sending command to turn off lights.")
                ser.write(BRIGHTNESS_OFF_CMD)
            elif brightness_switch_state == Brightness.LOW:
                print(f"[{current_time}] 👉 Sending command for low brightness.")
                ser.write(BRIGHTNESS_LOW_CMD)
            else:
                print(f"[{current_time}] 👉 Sending command for high brightness.")
                ser.write(BRIGHTNESS_HIGH_CMD)
            last_brightness_switch_state = brightness_switch_state
            command_sent = True

        if not command_sent and dew_heater_switch_state != last_dew_heater_switch_state:
            if dew_heater_switch_state == HeaterPower.OFF:
                print(f"[{current_time}] 👉 Sending command to turn off heater.")
                ser.write(HEATER_OFF_CMD)
            elif dew_heater_switch_state == HeaterPower.LOW:
                print(f"[{current_time}] 👉 Sending command for low heater power.")
                ser.write(HEATER_LOW_CMD)
            else:
                print(f"[{current_time}] 👉 Sending command for high heater power.")
                ser.write(HEATER_HIGH_CMD)
            last_dew_heater_switch_state = dew_heater_switch_state

        print(f"[{current_time}] #################")
        time.sleep(1)
except KeyboardInterrupt:
    print(f"\n[{current_time}] Read loop stopped by user. Exiting...")
except serial.SerialException:
    print(f"[{current_time}] Serial error: {traceback.format_exc()}")
except Exception:
    print(f"[{current_time}] An unexpected error occurred: {traceback.format_exc()}")
finally:
    GPIO.cleanup()
    if ser:
        ser.close()
        print(f"[{current_time}] Serial connection closed.")
