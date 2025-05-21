import RPi.GPIO as GPIO
import time

SWITCH_PINS = [
    (17, 27),  # Switch 1
    (22, 23),  # Switch 2
    (24, 25),  # Switch 3
]

GPIO.setmode(GPIO.BCM)


for pin_pair in SWITCH_PINS:
    for pin in pin_pair:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def read_switch(pin_a, pin_b):
    a = GPIO.input(pin_a)
    b = GPIO.input(pin_b)
    if a and not b:
        return "Position 1"
    elif not a and b:
        return "Position 2"
    elif not a and not b:
        return "Center (OFF)"
    else:
        return "Invalid (both HIGH)"


try:
    while True:
        for idx, (pin_a, pin_b) in enumerate(SWITCH_PINS):
            pos = read_switch(pin_a, pin_b)
            print(f"Switch {idx+1}: {pos}")
        print("---")
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
