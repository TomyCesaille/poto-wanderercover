import re
from typing import Never


def _parse_wanderer_status_message(status_message: Never) -> dict:
    """
    Parse the status message from the WandererCover device.

    Format: WandererCoverV4A***A***A***A***A***A***A***A
    Example: WandererCoverV4A20250506A15.00A280.00A14.41A12.78A0A50A1A
    1. Firmware version
    2. Close position set (°)
    3. Open position set (°)
    4. Current position (°)
    5. Input voltage (V)
    6. Flat panel brightness
    7. Dew heater power

    Returns a dictionary with the parsed values or `{"done": True}` for done feedback or None if parsing fails.
    """
    if isinstance(status_message, bytes):
        status_message = status_message.decode("utf-8", errors="replace")
    else:
        print(f"[STATUS] ERROR: Not an expected bytes type: {type(status_message)}")
        return None

    # Not expecting multiple messages as we're using ser.readline(), but handling last message in case.
    message = status_message.strip().split("\r\n")[-1]

    # It's undocumented, but the device sends "done" when it has finished moving the lid.
    if message == "done":
        return {"done": True}

    pattern = r"([^A]*)A([^A]*)A([^A]*)A([^A]*)A([^A]*)A([^A]*)A([^A]*)A([^A]*)A([^A]*)"
    match = re.search(pattern, message)

    if match:
        status = {
            "hardware": match.group(1),
            "firmware": match.group(2),
            "close_position": match.group(3),
            "open_position": match.group(4),
            "current_position": match.group(5),
            "voltage": match.group(6),
            "brightness": match.group(7),
            "heater": match.group(8),
            "undocumented_thing": match.group(9),  # Yeah it's there, but what is it?
        }
        return status
    else:
        print(f"[STATUS] ERROR: Could not parse received status: {message}")
        return None


def print_wanderer_status_message(status_message: Never) -> None:
    """
    Print the status message from the WandererCover device in a nice format.
    """
    parsed_data = _parse_wanderer_status_message(status_message)

    if not parsed_data:
        print("Could not parse status message.")
        return

    # Only print the latest status if multiple messages are received
    status = parsed_data[-1]

    print("\n=== WandererCover Status ============")
    print(f"Hardware:         {status['hardware']}")
    print(f"Firmware Version: {status['firmware']}")
    print(f"Close Position:   {status['close_position']}°")
    print(f"Open Position:    {status['open_position']}°")
    print(f"Current Position: {status['current_position']}°")
    print(f"Input Voltage:    {status['voltage']}")
    print(f"Flat Panel:       {status['brightness']}")
    print(f"Dew Heater:       {status['heater']}")
    print(f"Undocumented:     {status['undocumented_thing']}")
    print("=======================================\n")


def print_switches_state(
    openclose_switch_state: str,
    brightness_switch_state: str,
    dew_heater_switch_state: str,
) -> None:
    """
    Print the input state of the WandererCover device.
    """
    print("\n=== WandererCover Status ============")
    print(f"Open/Close Switch: {openclose_switch_state}")
    print(f"Brightness Switch: {brightness_switch_state}")
    print(f"Dew Heater Switch: {dew_heater_switch_state}")
    print("=======================================\n")
