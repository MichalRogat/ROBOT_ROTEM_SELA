from enum import Enum
from smbus2 import SMBus

# OPCODES
STARTPWM = 0
STOPPWM = 1
SETGPIO = 2
READADC = 3
GETGPIO = 4

HIGH = 1
LOW = 0

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

        packet2 = Packet(SETGPIO, [driver.pins[1], driver.pwm])
        sendPacketOrDebug(packet2, driver.I2CAddress)

        if (driver.isUglyDriver is True):
            packet3 = Packet(SETGPIO, [driver.pins[2], driver.extra])
            sendPacketOrDebug(packet3, driver.I2CAddress)
        else:
            packet4 = Packet(STARTPWM, [driver.pins[2], driver.extra])
            sendPacketOrDebug(packet4, driver.I2CAddress)
        
        packet1 = Packet(SETGPIO, payload=[driver.pins[0] ,driver.gpio])
        sendPacketOrDebug(packet1, driver.I2CAddress)

    @classmethod
    def callDigitalGpioFunction(cls, motor):
        # This method controls generic digital gpio
        packet = Packet(SETGPIO, payload=[motor.pin, motor.gpio])
        sendPacketOrDebug(packet, motor.I2CAddress)
        print(packet, motor.I2CAddress)
         
    @classmethod
    def callReadADC(cls, motor):
        packet = Packet(READADC, payload=[motor.adcPin])
        sendPacketOrDebug(packet, motor.I2CAddress)
        print(packet, motor.I2CAddress)