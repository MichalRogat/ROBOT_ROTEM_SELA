from enum import Enum
import smbus2
import warnings

# OPCODES
STARTPWM = 0
STOPPWM = 1
SETGPIO = 2
READADC = 3
GETGPIO = 4

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

    def to_bytes_array(self):
        # Serialize each attribute of the Packet struct
        self.SOT = self.SOT.to_bytes(1, 'little')
        self.opcode = self.opcode.to_bytes(1, 'little')
        self.payload_length = self.payload_length.to_bytes(1, 'little')
        self.payload = bytearray(self.payload)
        self.checksum = self.checksum.to_bytes(1, 'little')

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

# These pins are currently outdated
class MotorNumbers(Enum):
    P1, T1, D1 = 0, 1, 2
    E1, E2 = 3, 4
    T2, D3, T3 = 5, 6, 7
    E3, E4 = 8, 9
    P2, T4, D2 = 10, 11, 12

class Functions():

    class StartMotor():
        gpioValue = HIGH
        pwmSpeed = 90
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



motorPins = [[0, 0, 0], [2, 5, 3], [7, 6, 9], [12, 11, 10]]

class MotorDriver:

    def sendPacketOrDebug(self, packet:Packet, bus:smbus2.SMBus):
        if DEBUG:
            warnings.warn("smbus is not instanitiated")
            print(packet)
            return packet
        else:
            bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet.to_bytes_array())

    @classmethod
    def callFunction(self, func, motorNum, bus):
        gpioPacket = Packet(SETGPIO, payload=[motorPins[motorNum][0] ,func.gpioValue])
        self.sendPacketOrDebug(self, gpioPacket, bus)

        pwmPacket = Packet(STARTPWM, [motorPins[motorNum][1], func.pwmSpeed])
        self.sendPacketOrDebug(self, pwmPacket, bus)

        if (motorNum != 3):
            pwm2Packet = Packet(STARTPWM, [motorPins[motorNum][2], func.pwm2Value])
            self.sendPacketOrDebug(self, pwm2Packet, bus)
        else:
            gpio2Packet = Packet(SETGPIO, [motorPins[motorNum][2], func.gpio2Value])
            self.sendPacketOrDebug(self, gpio2Packet, bus)
        print(f"Started motor {motorNum}")

    @classmethod
    def stopPumps(self, bus:smbus2.SMBus=None):
        for pumpMotor in PumpMotors:
            MotorDriver.callFunction(Functions.StopMotor, motorNum=pumpMotor)

    @classmethod
    def stopAllMotors(self, bus:smbus2.SMBus=None):
        for robotPinNumber in MotorNumbers:
            MotorDriver.callFunction(Functions.StopMotor, motorNum=robotPinNumber)

    # @classmethod
    # def getMotorCurrent(self, motor:NanoPins, bus:smbus2.SMBus=None):
    #     packet = Packet(Opcodes.readAdc, payload=[motor.value])
    #     if bus is not None:
    #         bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet.to_bytes_array())
    #     else:
    #         warnings.warn("smbus is not instanitiated")
    #         return packet

    # @classmethod
    # def startMotor(self, motorNum, speed:int, bus:smbus2.SMBus):
    #     gpioPacket = Packet(SETGPIO, payload=[motorPins[motorNum][0] ,HIGH])
    #     self.sendPacketOrDebug(gpioPacket, bus)

    #     pwmPacket = Packet(STARTPWM, [motorPins[motorNum][1], speed])
    #     self.sendPacketOrDebug(pwmPacket, bus)

    #     if (motorNum != 3):
    #         pwm2Packet = Packet(STARTPWM, [motorPins[motorNum][2], LOW])
    #         self.sendPacketOrDebug(pwm2Packet, bus)
    #     else:
    #         gpio2Packet = Packet(SETGPIO, [motorPins[motorNum][2], HIGH])
    #         self.sendPacketOrDebug(gpio2Packet, bus)
    #     print(f"Started motor {motorNum}")

    # @classmethod    
    # def stopMotor(self, motorNum, bus:smbus2.SMBus=None):
    #     gpioPacket = Packet(opcode = SETGPIO, payload=[motorPins[motorNum][0], LOW])
    #     self.sendPacketOrDebug(gpioPacket, bus)
        
    #     pwmPacket = Packet(STARTPWM, [motorPins[motorNum][1], LOW])
    #     self.sendPacketOrDebug(pwmPacket, bus)
        

    # @classmethod
    # def stopAllMotors(self, bus:smbus2.SMBus=None):
    #     for robotPinNumber in NanoPins:
    #         packet = Packet(opcode=Opcodes.stopPwm, payload=[robotPinNumber.value])
    #         if bus is not None:
    #             bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet.to_bytes_array().to_bytes_array())
    #         else:
    #             warnings.warn("smbus is not instanitiated")
    #             return packet



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
    MotorDriver.callFunction(Functions.StartMotor, 1,bus)
    MotorDriver.callFunction(Functions.StartMotor,2,bus)
    MotorDriver.callFunction(Functions.StartMotor,3,bus)
    MotorDriver.callFunction(Functions.StopMotor, 1,bus)
    MotorDriver.callFunction(Functions.StopMotor, 2,bus)
    MotorDriver.callFunction(Functions.StopMotor, 3,bus)
    