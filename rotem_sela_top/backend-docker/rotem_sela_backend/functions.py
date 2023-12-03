from smbus2 import SMBus
import time
from threading import Lock
import traceback
import ctypes

DEBUG = False
# OPCODES
SET_DRIVER_STATE = 0

bus = SMBus(1)

lock = Lock()

class Packet:
    def __init__(self, payload:list, pIdx=0):
        self.SOT = 0xAA
        self.opcode = SET_DRIVER_STATE
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
        for data in self.payload:
            if data < 0:
                checksum += ctypes.c_ubyte(data).value
            else:
                checksum += data
        return checksum%255
    
    def to_array(self):
        return [self.SOT, self.packetIdx,  self.opcode, self.payload_length] + self.payload + [self.checksum]

    def __repr__(self) -> str:
        return f"op: {self.opcode}, pIdx: {self.packetIdx}, payload_length: {self.payload_length}, payload: {self.payload}" 

pIdx = {}
def i2c_write(i2c_addr, data):
    if DEBUG:
        # print(data)
        pass
    else:
        try:
            lock.acquire()
            bus.write_i2c_block_data(i2c_addr=i2c_addr, register=0x1, data=data)
        finally:
            lock.release()
        
    return

def i2c_read(i2c_addr, len):
    res = None
    if DEBUG:
        pass
    else:
        lock.acquire()
        
        try:
            res = bus.read_i2c_block_data(i2c_addr=i2c_addr, register=0x1, length=len)
        finally:
            lock.release()
    
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

def callReadNano(trailers, nanoTelemetry, motors, debug=False):
    for trailer in trailers:
        try:
            global pIdx
            if trailer.I2CAddress not in pIdx:
                pIdx[trailer.I2CAddress] = {'rcv':-1, 'send':0}
            packet = trailer.GetState()
            i2c_write(trailer.I2CAddress, packet.to_array())

            if not debug:
                ret_byte = i2c_read(trailer.I2CAddress, 32)
                if ret_byte[0] == 170 and ret_byte[4] == 175:
                    continue
                nanoTelemetry['FullTank'+trailer.name] = int.from_bytes(ret_byte[0:2], byteorder='little')
                nanoTelemetry['drive'+trailer.name] = int.from_bytes(ret_byte[2:4], byteorder='little')
                nanoTelemetry['m2CS'] = int.from_bytes(ret_byte[4:6], byteorder='little')
                nanoTelemetry['m3CS'] = int.from_bytes(ret_byte[6:8], byteorder='little')
                nanoTelemetry['batteryRead'] = int.from_bytes(ret_byte[8:10], byteorder='little')
                nanoTelemetry['fault1'] = int.from_bytes(ret_byte[10:11], byteorder='little')
                nanoTelemetry['fault2'] = int.from_bytes(ret_byte[11:12], byteorder='little')
                nanoTelemetry['fault3'] = int.from_bytes(ret_byte[12:13], byteorder='little')

                nanoTelemetry['imu'+trailer.name] = [int.from_bytes(ret_byte[13:17], byteorder='little', signed=True)/10.0,
                                                        int.from_bytes(ret_byte[17:21], byteorder='little', signed=True)/10.0,
                                                        int.from_bytes(ret_byte[21:25], byteorder='little', signed=True)/10.0
                                                    ]
                pIdx[trailer.I2CAddress]['rcv'] = ret_byte[25]


            else:
                nanoTelemetry.update(fakeTelemetry(trailer, nanoTelemetry))

            # for motor in motors:
            #     nanoTelemetry["name of motor"] = motor.name
            #     nanoTelemetry["speed of motor"] = motor.speed
        except Exception:
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

if __name__ == "__main__":
    callReadNano("motor", "nanotelemetry", "motors", True)
    callReadNano()