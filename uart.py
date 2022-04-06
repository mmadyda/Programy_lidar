import serial
import sys

ser=serial.Serial(
port='COM12',
baudrate=115200,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)

if ser.isOpen():
    ser.close()
ser.open()


while True:#value > -1 and value < 10:
    try:
        data = str(ser.readline().decode('Ascii'))
        data = data.strip()
        print(data)


        #sys.stdout.write('%s\r' % data )
        #sys.stdout.flush()
    except:
        print("Error")

