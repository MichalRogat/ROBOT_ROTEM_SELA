from smbus2 import SMBus
import time
from threading import Lock
import traceback
import ctypes
import serial
from gpiozero import CPUTemperature

DEBUG = True
# OPCODES
SET_DRIVER_STATE = 0
CALIBRATE = 1
CONFIG = 2
TELEMETRY = 3
ACK = 255

bus = SMBus(1)
ser = serial.Serial ("/dev/ttyS0", 115200) 
ser.timeout = 0.05
lock = Lock()

class Packet:
    def __init__(self, payload:list=[], opcode=SET_DRIVER_STATE, pIdx=0):
        self.SOT = 0xAA
        self.opcode = opcode
        self.packetIdx = pIdx
        self.payload_length = len(payload)
        self.payload = payload
        self.checksum = self.calculate_checksum()

    def calculate_checksum(self):
        checksum = 0
        checksum += self.SOT
        checksum += self.opcode
        checksum += self.packetIdx
        checksum += self.payload_length
        for i in range(0,self.payload_length):
            data = self.payload[i]
            if data < 0:
                checksum += ctypes.c_ubyte(data).value
            else:
                checksum += data
        return checksum&0xFF
    
    def calculate_checksum_rcv(self):
        checksum = 0
        checksum += self.SOT
        checksum += self.opcode
        checksum += self.packetIdx
        checksum += self.payload_length
        for i in range(0,self.payload_length):
            data = self.payload[i]
            # if data < 0:
            #     checksum += ctypes.c_ubyte(data).value
            # else:
            checksum += data
        return checksum&0xFF
    
    def to_array(self):
        res = [self.SOT, self.packetIdx,  self.opcode, self.payload_length] + self.payload + [self.checksum]

        for i in range(0, len(res)):
            if res[i] < 0:
                res[i] = ctypes.c_ubyte(res[i]).value

        return res

    def __repr__(self) -> str:
        return f"op: {self.opcode}, pIdx: {self.packetIdx}, payload_length: {self.payload_length}, payload: {self.payload}" 

pIdx = {}

def serial_write(data):
     ser.write(data)

def serial_read(len):
    res = ser.read(len)
    return res
    # byteCount = 0
    # while ser.inWaiting():
    #     received_data = ser.read()              #read serial port
    #     sleep(0.03)
    #     data_left = ser.inWaiting()             #check for remaining byte
    #     received_data += ser.read(data_left)
    #     print (received_data)                   #print received data
    #     ser.write(received_data)

def i2c_write(i2c_addr, data):
    
    try:
        # lock.acquire()
        bus.write_i2c_block_data(i2c_addr=i2c_addr, register=0x1, data=data)
    finally:
        # lock.release()
        pass
    return

def i2c_read(i2c_addr, len):
    res = None
    
    # lock.acquire()
    
    try:
        res = bus.read_i2c_block_data(i2c_addr=i2c_addr, register=0x1, length=len)
    finally:
        # lock.release()
        pass
    
    return res

def fakeTelemetry(trailer, nanoTelemetry):
    nanoTelemetry['FullTank'+trailer.name] = 0
    nanoTelemetry['drive'+trailer.name] = 0
    nanoTelemetry['m2CS'] = 0
    nanoTelemetry['m3CS'] = 0
    nanoTelemetry['batteryRead'] = 0
    nanoTelemetry['fault1'] = 0
    nanoTelemetry['fault2'] = 0
    nanoTelemetry['fault3'] = 0
    nanoTelemetry['imu'+trailer.name] = [0,0,0]
    return nanoTelemetry

def readPacket(timeout=1):
    idx = 0
    startByte = False
    p = Packet()
    ts = time.time()
    while ser.in_waiting > 0 or time.time() - ts < timeout:
        ret_byte = serial_read(1)

        if len(ret_byte) == 0:
            continue

        # print(ret_byte[0])
        
        if not startByte and ret_byte[0] == 0xAA:
            startByte = True
            p.payload = []

        if startByte:
            # print("Byte "+str(ret_byte[0]))
            if idx == 0:
                p.SOT = ret_byte[0]
            elif idx == 1:
                p.packetIdx = ret_byte[0]
            elif idx == 2:
                p.opcode = ret_byte[0]
            elif idx == 3:
                p.payload_length = ret_byte[0]
                if p.payload_length > 80:
                    startByte = False
                    continue
            elif idx <= p.payload_length + 3:
                p.payload.append(ret_byte[0])
            
            else:
                # print("PAYLOAD "+str(idx) +" "+str(p.payload_length))
                chk = p.calculate_checksum()
                if chk == ret_byte[0]:
                    return p
                else:
                    print("Wrong checksum "+str(chk))
                    startByte = False
            idx += 1
    return None        
        
lastTempPrint = 0

def callReadNano(trailers, nanoTelemetry, motors, debug=False):
    global lastTempPrint
    if lastTempPrint == 0:
        lastTempPrint = time.time()

    if time.time() - lastTempPrint > 2:
        print("CPU temp "+str(CPUTemperature().temperature))
        lastTempPrint = time.time()

    for trailer in trailers:
        try:
            global pIdx
            if trailer.I2CAddress not in pIdx:
                pIdx[trailer.I2CAddress] = {'rcv':-1, 'send':0}
            packet = trailer.GetState()

            if not debug:

                
                # print("I@C trans")
                # i2c_write(trailer.I2CAddress, packet.to_array())
                # ret_byte = i2c_read(trailer.I2CAddress, 32)

                serial_write(packet.to_array())
                p = readPacket()
                if p is not None:
                    if p.opcode == ACK:
                        # print("ACK "+str(round(time.time()*1000)))
                        continue
                    elif p.opcode == TELEMETRY:
                        # print("Telemetry "+str(round(time.time()*1000)))
                        nanoTelemetry['FullTank'+trailer.name] = int.from_bytes(p.payload[0:2], byteorder='little')
                        nanoTelemetry['m1CS'+trailer.name] = int.from_bytes(p.payload[2:4], byteorder='little')
                        nanoTelemetry['m2CS'+trailer.name] = int.from_bytes(p.payload[4:6], byteorder='little')
                        nanoTelemetry['m3CS'+trailer.name] = int.from_bytes(p.payload[6:8], byteorder='little')
                        nanoTelemetry['batteryRead'+trailer.name] = int.from_bytes(p.payload[8:10], byteorder='little')
                       
                        nanoTelemetry['imu'+trailer.name] = [int.from_bytes(p.payload[10:14], byteorder='little', signed=True)/10.0,
                                                        int.from_bytes(p.payload[14:18], byteorder='little', signed=True)/10.0,
                                                        int.from_bytes(p.payload[18:22], byteorder='little', signed=True)/10.0
                                                    ]
                        
                        nanoTelemetry['accel'+trailer.name] = [int.from_bytes(p.payload[22:26], byteorder='little', signed=True)/10.0,
                                                        int.from_bytes(p.payload[26:30], byteorder='little', signed=True)/10.0,
                                                        int.from_bytes(p.payload[30:34], byteorder='little', signed=True)/10.0
                                                    ]

                        nanoTelemetry['gyro'+trailer.name] = [int.from_bytes(p.payload[34:38], byteorder='little', signed=True)/10.0,
                                                        int.from_bytes(p.payload[38:42], byteorder='little', signed=True)/10.0,
                                                        int.from_bytes(p.payload[42:46], byteorder='little', signed=True)/10.0
                                                    ]
                        

                        print("Telemetry "+trailer.name)
                        print(str(nanoTelemetry))
                else:
                    print("Error "+str(trailer.I2CAddress))
                    continue
                

            else:
                nanoTelemetry.update(fakeTelemetry(trailer, nanoTelemetry))

            for motor in motors:
                nanoTelemetry["name of motor"] = motor.name
                nanoTelemetry["speed of motor"] = motor.speed

        except Exception as e:
            print(str(trailer.I2CAddress))
            traceback.print_exc()
    
def startCalibration():
    packet = Packet([1], 0x11) # start calibration
    packet.opcode = 1 # calibration
    packet.checksum = packet.calculate_checksum()
    i2c_write(0x11, packet.to_array())

def stopCalibration():
    packet = Packet([0], 0x11) # start calibration
    packet.opcode = 1 # calibration
    packet.checksum = packet.calculate_checksum()
    i2c_write(0x11, packet.to_array())
