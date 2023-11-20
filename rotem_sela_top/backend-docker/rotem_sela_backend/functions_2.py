from enum import Enum
from smbus2 import SMBus
from minimu import MinIMU_v5_pi

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


class ArduinoAddress(Enum):
    Arduino1 = 0x11
    Arduino2 = 0x12
    Arduino3 = 0x13
    Arduino4 = 0x14
    Arduino5 = 0x15


class Packet:
    def __init__(self, opcode, payload: list):
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
        return checksum % 255

    def to_array(self):
        return [self.SOT, self.opcode, self.payload_length] + self.payload + [self.checksum]

    def __repr__(self) -> str:
        return f"{self.SOT}, {self.opcode}, {self.payload_length}, {self.payload}, {self.checksum}"


def sendPacketOrDebug(packet: Packet, i2c_addr):
    if DEBUG:
        print(f"packet: {packet}")
    else:
        print(f"i2caddr: {i2c_addr}, packet:{packet}")
        bus.write_i2c_block_data(
            i2c_addr=i2c_addr, register=0x1, data=packet.to_array())


class GenericFunctions:

    @classmethod
    def callDriverFunction(cls, driver):
        # Controls generic driver which controls over 3 pins.
        # func - the function to perform e.g. stopMotor startMotor
        # motor - the motor to stop

        packet2 = Packet(SETGPIO, [driver.pins[SLEEP], driver.gpio])
        sendPacketOrDebug(packet2, driver.I2CAddress)
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
    def callReadADC(cls, motor):
        packet = Packet(READADC, payload=[motor.adcPin])
        sendPacketOrDebug(packet, motor.I2CAddress)
        print(packet, motor.I2CAddress)

    @classmethod
    def readIMU(self, address):
        IMUPacket_R = Packet(READIMU, payload=[])
        sendPacketOrDebug(IMUPacket_R, address)
