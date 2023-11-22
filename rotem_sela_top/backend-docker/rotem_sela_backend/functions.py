from smbus2 import SMBus
import json
import struct
import time
from threading import Lock
# from robot_main2 import info
# import threading

# OPCODES
STARTPWM = 0
STOPPWM = 1
SETGPIO = 2
READADC = 3
GETGPIO = 4
READIMU = 5
WRITEIMU = 6

HIGH = 1
LOW = 0

IN1 = 0
IN2 = 1
SLEEP = 2

DEBUG = False
if not DEBUG:
    bus = SMBus(1)

lock = Lock()
    
class Packet:
    def __init__(self, opcode, payload:list):
        self.SOT = 0xAA
        self.opcode = opcode
        self.payload_length = len(payload)
        self.payload = payload
        self.checksum = self.calculate_checksum()

    def calculate_checksum(self):
        checksum = 0
        checksum += self.SOT
        checksum += self.opcode
        checksum += self.payload_length
        for data in self.payload:
            checksum += data
        return checksum%255
    
    def to_array(self):
        return [self.SOT, self.opcode, self.payload_length] + self.payload + [self.checksum]

    def __repr__(self) -> str:
        return f"{self.SOT}, {self.opcode}, {self.payload_length}, {self.payload}, {self.checksum}" 

def i2c_write(i2c_addr, data):
    retries = 0
    lock.acquire()
    while retries < 4:
        try:
            
            bus.write_i2c_block_data(i2c_addr=i2c_addr, register=0x1, data=data)
            break
        except Exception as e:
            retries = retries + 1
            time.sleep(0.01)

    lock.release()
    if retries >= 4:
        print("Failed i2c")
    return

def i2c_read(i2c_addr, len):
    res = None
    retries = 0
    lock.acquire()
    while retries < 4:
        try:
            
            res = bus.read_i2c_block_data(i2c_addr=i2c_addr, register=0x1, length=len)
            break
        except Exception as e:
            retries = retries + 1
            time.sleep(0.01)

    lock.release()
    if res is None:
        print("Failed i2c read")
    return res

def sendPacketOrDebug(packet:Packet, i2c_addr):
    if DEBUG:
        print(f"packet: {packet}")
    else:
        # print(f"i2caddr: {i2c_addr}, packet:{packet}")
       
        i2c_write(i2c_addr, packet.to_array())
       

class GenericFunctions:

    @classmethod
    def callDriverFunction(cls, driver):
        # Controls generic driver which controls over 3 pins.
        # func - the function to perform e.g. stopMotor startMotor
        # motor - the motor to stop

        packet2 = Packet(SETGPIO, [driver.pins[SLEEP], driver.gpio])
        sendPacketOrDebug(packet2, driver.I2CAddress)
        if driver.IN1type is None:
            return
        packet3 = Packet(driver.IN1type, [driver.pins[IN1], driver.IN1])
        sendPacketOrDebug(packet3, driver.I2CAddress)
        packet4 = Packet(driver.IN2type, [driver.pins[IN2], driver.IN2])
        sendPacketOrDebug(packet4, driver.I2CAddress)
       

    @classmethod
    def callDigitalGpioFunction(cls, motor):
        # This method controls generic digital gpio
        packet = Packet(SETGPIO, payload=[motor.pin, motor.gpio])
        sendPacketOrDebug(packet, motor.I2CAddress)
        print(packet, motor.I2CAddress)

    @classmethod
    def callReadNano(cls, trailers, nanoTelemetry):
        for trailer in trailers:
            try:
                ret_byte = i2c_read(trailer.I2CAddress, 32)

                nanoTelemetry['FullTank'+trailer.name] = int.from_bytes(ret_byte[0:2], byteorder='little') # - 2B
                nanoTelemetry['drive'+trailer.name] = int.from_bytes(ret_byte[2:4], byteorder='little') #- 2B
                nanoTelemetry['m2CS'] = int.from_bytes(ret_byte[4:6], byteorder='little') # - 2B
                nanoTelemetry['m3CS'] = int.from_bytes(ret_byte[6:8], byteorder='little') #- 2B
                nanoTelemetry['batteryRead'] = int.from_bytes(ret_byte[8:10], byteorder='little') #- 2B
                nanoTelemetry['fault1'] = int.from_bytes(ret_byte[10:11], byteorder='little') #- 1B
                nanoTelemetry['fault2'] = int.from_bytes(ret_byte[11:12], byteorder='little') # 1B
                nanoTelemetry['fault3'] = int.from_bytes(ret_byte[12:13], byteorder='little') # - 1B
                integer_part_roll = ret_byte[13]
                fraction_part_roll = ret_byte[14]
                integer_part_pitch = ret_byte[15]
                fraction_part_pitch = ret_byte[16]
                integer_part_yaw = ret_byte[17]
                fraction_part_yaw = ret_byte[18]
                
                nanoTelemetry['imu'+trailer.name]=[integer_part_roll+fraction_part_roll/10.0,
                                          integer_part_pitch+fraction_part_pitch/10.0,
                                          integer_part_yaw+fraction_part_yaw/10.0                                     

                ]
                
            except Exception as e:
                # print(f"Trailer {trailer.name} {e}")
                pass
            finally:
                time.sleep(0.05)
        print(nanoTelemetry)
    # @classmethod
    # def callReadNano(cls, trailers):
    #     for trailer in trailers:
    #         try:
    #             ret_byte = bus.read_i2c_block_data(trailer.I2CAddress, 1, 32)
    #             info = {}

    #             info['FullTank'+trailer.name] = int.from_bytes(ret_byte[0:2], byteorder='little') # - 2B
    #             info['drive'+trailer.name] = int.from_bytes(ret_byte[2:4], byteorder='little') #- 2B
    #             info['m2CS'] = int.from_bytes(ret_byte[4:6], byteorder='little') # - 2B
    #             info['m3CS'] = int.from_bytes(ret_byte[6:8], byteorder='little') #- 2B
    #             info['batteryRead'] = int.from_bytes(ret_byte[8:10], byteorder='little') #- 2B
    #             info['fault1'] = int.from_bytes(ret_byte[10:11], byteorder='little') #- 1B
    #             info['fault2'] = int.from_bytes(ret_byte[11:12], byteorder='little') # 1B
    #             info['fault3'] = int.from_bytes(ret_byte[12:13], byteorder='little') # - 1B
    #             # info['imu'+trailer.name]=[
    #             #                         struct.unpack('>f', bytes(ret_byte[13:17]))[0], #yaw
    #             #                         struct.unpack('>f', bytes(ret_byte[17:21]))[0], #pitch
    #             #                         struct.unpack('>f', bytes(ret_byte[21:25]))[0] # roll
    #             #                         ]
    #             integer_part_roll = ret_byte[13]
    #             fraction_part_roll = ret_byte[14]
    #             integer_part_pitch = ret_byte[15]
    #             fraction_part_pitch = ret_byte[16]
    #             integer_part_yaw = ret_byte[17]
    #             fraction_part_yaw = ret_byte[18]
                
    #             info['imu'+trailer.name]=[integer_part_roll+fraction_part_roll/10.0,
    #                                       integer_part_pitch+fraction_part_pitch/10.0,
    #                                       integer_part_yaw+fraction_part_yaw/10.0                                     

    #             ]
    #             # print(f"info from functions: {json.dumps(info)}")
    #             print(f"info from functions: {json.dumps(info)}")
    #             # nanoTelemetry.update(info)
    #         except Exception as e:
    #             print(f"Trailer {trailer.name} {e}")
    #         finally:
    #             time.sleep(0.05)
        
