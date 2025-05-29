import time
import serial

ser = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=19200,
    bytesize=8,
    parity=serial.PARITY_NONE,
    stopbits=1,
    timeout=5,
)
print(f"Reading on port {ser.portstr} at {ser.baudrate} baud")

try:
    print("StartFing serial read loop. Press Ctrl+C to stop.")

    while True:
        data = ser.readline()

        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

        if data:
            try:
                decoded_data = data.decode("utf-8").strip()
                print(f"[{current_time}] Received: {decoded_data}")
            except UnicodeDecodeError:
                print(f"[{current_time}] Received (hex): {data.hex()}")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nRead loop stopped by user")

finally:
    ser.close()
    print("Serial port closed")
