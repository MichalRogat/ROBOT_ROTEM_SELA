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

DEBUG = True
if not DEBUG:
    bus = SMBus(1)

class ArduinoAddress(Enum):
    Arduino0 = 0x55
    
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
        return f"{self.SOT},\t {self.opcode},\t {self.payload_length},\t {self.payload},\t {self.checksum}"

def sendPacketOrDebug(packet:Packet):
    if DEBUG:
        print(f"packet: {packet}")
    else:
        bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0.value, register=0x1, data=packet.to_array())

class GenericFunctions:

    @classmethod
    def callDriverFunction(cls, driver):
        # Controls generic driver which controls over 3 pins.
        # func - the function to perform e.g. stopMotor startMotor
        # motor - the motor to stop

        packet2 = Packet(STARTPWM, [driver.pins[1], driver.pwm])
        sendPacketOrDebug(packet2)

        if (driver.isUglyDriver is True):
            packet3 = Packet(SETGPIO, [driver.pins[2], driver.extra])
            sendPacketOrDebug(packet3)
        else:
            packet4 = Packet(STARTPWM, [driver.pins[2], driver.extra])
            sendPacketOrDebug(packet4)
        
        packet1 = Packet(SETGPIO, payload=[driver.pins[0] ,driver.gpio])
        sendPacketOrDebug(packet1)

    @classmethod
    def callDigitalGpioFunction(cls, motor):
        # This method controls generic digital gpio
        packet = Packet(SETGPIO, payload=[motor.pin, motor.gpio])
        sendPacketOrDebug(packet)
        print(packet)
         
    @classmethod
    def callReadADC(cls, motor):
        packet = Packet(READADC, payload=[motor.adcPin])
        sendPacketOrDebug(packet)
        print(packet)