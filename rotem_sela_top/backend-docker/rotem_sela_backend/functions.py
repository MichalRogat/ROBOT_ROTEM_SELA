from smbus2 import SMBus
import json
import struct
from datetime import datetime
from robot_remote_control import CommandOpcode
import time

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

def sendPacketOrDebug(packet:Packet, i2c_addr):
    if DEBUG:
        print(f"packet: {packet}")
    else:
        print(f"i2caddr: {i2c_addr}, packet:{packet}")
        bus.write_i2c_block_data(i2c_addr=i2c_addr, register=0x1, data=packet.to_array())

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
    def callReadNano(cls, trailers):

        while(True):
            for trailer in trailers:
                try:
                    ret_byte = bus.read_i2c_block_data(trailer.I2CAddress, 1, 32)
                    res = {}
                    res['FullTank'+trailer.name] = int.from_bytes(ret_byte[0:2], byteorder='little') # - 2B
                    res['drive'+trailer.name] = int.from_bytes(ret_byte[2:4], byteorder='little') #- 2B
                    res['m2CS'] = int.from_bytes(ret_byte[4:6], byteorder='little') # - 2B
                    res['m3CS'] = int.from_bytes(ret_byte[6:8], byteorder='little') #- 2B
                    res['batteryRead'] = int.from_bytes(ret_byte[8:10], byteorder='little') #- 2B
                    res['fault1'] = int.from_bytes(ret_byte[10:11], byteorder='little') #- 1B
                    res['fault2'] = int.from_bytes(ret_byte[11:12], byteorder='little') # 1B
                    res['fault3'] = int.from_bytes(ret_byte[12:13], byteorder='little') # - 1B
                    res['imu'+trailer.name]=[
                                                    struct.unpack('>f', bytes(ret_byte[13:17]))[0], #yaw
                                                    struct.unpack('>f', bytes(ret_byte[17:21]))[0], #pitch
                                                    struct.unpack('>f', bytes(ret_byte[21:25]))[0] # roll
                    ]
                    res["opcode"] = CommandOpcode.telemetric.name,
                    res["joint1"] = "not implemented" # self.a2d.values[6],
                    res["activePump"] = "not implemented" # self.activePump,
                    res["pumpingNow"] = "not implemted," # self.isPumpingNow # 
                    res["Spare2"] = 4096,
                    res["Spare3"] = 4096,
                    res["Spare4"] = 4096,
                    res["Spare5"] = 4096,
                    res["Spare6"] = 4096,
                    res["Spare7"] = 4096,
                    res["Camera-F1"] = True,
                    res["Camera-S1"] = True,
                    res["Camera-F2"] = True,
                    res["Camera-S2"] = True,
                    res["Camera-F3"] = True,
                    res["Camera-S3"] = True,
                    res["Camera-F4"] = True,
                    res["Camera-S4"] = True,
                    res["isFlip"]: "not implemeted" #self.isFlip,
                    res["isToggle"]: "not implemted " # self.isToggle
                    # print(json.dumps(res))
                except Exception as e:
                    print(f"{trailer} {e}")
                finally:
                    time.sleep(0.1)
        