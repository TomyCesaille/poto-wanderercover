import time
import serial

ser = serial.Serial('/dev/ttyUSB0', 19200)
ser.write(b'9999')
time.sleep(3)
ser.write(b'255')
time.sleep(3)
ser.write(b'9999')
