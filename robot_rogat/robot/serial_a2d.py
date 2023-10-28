import queue
import serial
import struct
from time import sleep
    

class SerialA2D():
    def __init__(self):
        self.ser = serial.Serial ("/dev/ttyS0", 115200)    #Open port with baud rate
        self.values = []

    def listen(self):    
        while True:
                    received_data = self.ser.read(1)              #read serial port
                    if received_data[0] == 0xaa:
                        received_data = self.ser.read(1)    
                        if received_data[0] == 0xde:
                            received_data = self.ser.read(21)
                            self.values = list(struct.unpack('10h', received_data[0:20]))
                        else:
                            continue
                    else:
                        continue
    
if __name__ == "__main__":
    x = SerialA2D()
    x.listen()