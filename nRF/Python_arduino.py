import serial

import serial
import time
arduino = serial.Serial(port='COM4', baudrate=9600, timeout=.1)



"""


any code her


"""
for i in range(300):
    x = "1234567890abcdefghijklmnopqrstuvwxyz"
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.3)