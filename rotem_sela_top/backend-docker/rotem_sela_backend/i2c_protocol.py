from enum import Enum
import smbus2
import warnings
import Entity

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

DEBUG = False

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
        return f"SOT:{self.SOT},\t opcode:{self.opcode},\t payload_length:{self.payload_length},\t payload:{self.payload},\t checksum:{self.checksum}"

class ArduinoAddress(Enum):
    Arduino0 = 0x55
    # Fake addresses
    # Arduino1 = 1
    # Arduino2 = 2
    # Arduino3 = 3
    # Arduino4 = 4

# These pins are outdated
class PumpMotors(Enum):
    P1, P2 = 0, 10 

class Functions():
    # First in counter clock
    class MotorRun():
        gpioValue = HIGH
        pwmSpeed = 50
        pwm2Value = LOW
        gpio2Value = HIGH
    class StopMotor():
        gpioValue = LOW
        pwmSpeed = 0
        pwm2Value = LOW
        gpio2Value = LOW
    class ReadADC():
        pass
    class ReadAnalog():
        pass
    class StopAllMotors():
        pass
    class StartPump():
        pass
    class StopPump():
        pass
    class ReadTank():
        pass
    class ReadBattery():
        pass
    class ReadOverCurrent():
        pass


# motorPins = [[0, 0, 0], [2, 5, 3], [7, 9, 6], [12, 11, 10]]

class MotorDriver:

    def sendPacketOrDebug(self, packet:Packet, bus:smbus2.SMBus):
        if DEBUG:
            warnings.warn("smbus is not instanitiated")
            print(packet)
            return packet
        else:
            bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0.value, register=1, data=packet.to_array())

    @classmethod
    def callMotorFunction(self, func, motor:Entity.IMotor, bus):
        gpioPacket = Packet(SETGPIO, payload=[motor.pins[0] ,func.gpioValue])
        self.sendPacketOrDebug(self, gpioPacket, bus)

        pwmPacket = Packet(STARTPWM, [motor.pins[1], func.pwmSpeed])
        self.sendPacketOrDebug(self, pwmPacket, bus)

        if (motor.isUglyFlag is True):
            pwm2Packet = Packet(STARTPWM, [motor.pins[2], func.pwm2Value])
            self.sendPacketOrDebug(self, pwm2Packet, bus)
        else:
            gpio2Packet = Packet(SETGPIO, [motor.pins[2], func.gpio2Value])
            self.sendPacketOrDebug(self, gpio2Packet, bus)
        print(f"Started motor")

    @classmethod
    def stopPumps(self, bus:smbus2.SMBus=None):
        for pumpMotor in PumpMotors:
            MotorDriver.callMotorFunction(Functions.StopMotor, motorNum=pumpMotor)

    # @classmethod
    # def stopAllMotors(self, bus:smbus2.SMBus=None):
    #     for robotPinNumber in MotorNumbers:
    #         MotorDriver.callMotorFunction(Functions.StopMotor, motorNum=robotPinNumber)

    # @classmethod
    # def getMotorCurrent(self, motor:NanoPins, bus:smbus2.SMBus=None):
    #     packet = Packet(Opcodes.readAdc, payload=[motor.value])
    #     if bus is not None:
    #         bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet.to_bytes_array())
    #     else:
    #         warnings.warn("smbus is not instanitiated")
    #         return packet

    # @classmethod
    # def getMotorFault(self, motor:NanoPins, bus:smbus2.SMBus=None):
    #     packet = Packet(Opcodes.setGpio, [motor.value])
    #     if bus is not None:
    #         bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet.to_bytes_array())
    #     else:
    #         warnings.warn("smbus is not instanitiated")
    #         return packet   


if __name__ == "__main__":
    bus = smbus2.SMBus(1)
    MotorDriver.callMotorFunction(Functions.MotorRun, motor=Entity.D1Motor, bus = bus)
    MotorDriver.callMotorFunction(Functions.StopMotor, motor=Entity.D1Motor, bus = bus)
    