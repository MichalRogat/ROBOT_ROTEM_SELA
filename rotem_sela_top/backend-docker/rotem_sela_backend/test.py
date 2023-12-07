import serial
#Serial takes two parameters: serial device and baudrate
ser = serial.Serial('/dev/ttyS0', 115200)

while True:
    if ser.in_waiting > 0:
        print(ser.read(ser.in_waiting), end=None)
