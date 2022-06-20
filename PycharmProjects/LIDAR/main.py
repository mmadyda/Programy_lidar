import sys

import serial
import time


ser = serial.Serial()
ser.port = "COM12"
ser.baudrate = 115200
ser.timeout = 1
ser.setDTR(False)
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE


ser.open()




if ser.isOpen() == False:
    ser.open() # open serial port if not open

while True:
    data = ser.readline().decode('utf-8')
    print(data, end='\r')
    print('\r')

ser.close() # close serial port