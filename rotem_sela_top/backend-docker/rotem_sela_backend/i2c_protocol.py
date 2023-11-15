from enum import Enum
import array
import smbus2
import warnings

class Opcodes(Enum):
    startPwm = 0
    stopPwm = 1
    setGpio = 2
    readAdc = 3
    getGPio = 4

class Packet:
    def __init__(self, opcode:Opcodes, payload:array):
        self.SOT = 0xAA
        self.opcode = opcode
        self.payload_length = len(payload)
        self.payload = payload
        self.calculate_checksum()

    def calculate_checksum(self):
        checksum = 0
        checksum += self.SOT
        checksum += self.opcode.value
        checksum += ((self.payload_length) & 0xFF)  # LSB of length
        checksum += ((self.payload_length) >> 8)  # MSB of length
        for data in self.payload:
            checksum += data
        self.checksum = (~checksum) & 0xFF

    def __repr__(self) -> str:
        return f"SOT:{self.SOT},\t opcode:{self.opcode},\t payload_length:{self.payload_length},\t payload:{self.payload},\t checksum:{self.checksum}"

class ArduinoAddress(Enum):
    Arduino0 = 0
    Arduino1 = 1
    Arduino2 = 2
    Arduino3 = 3
    Arduino4 = 4

class PumpMotors(Enum):
    P1, P2 = 0, 10 

class RobotPinNumbers(Enum):
    P1, T1, D1 = 0, 1, 2
    E1, E2 = 3, 4
    T2, D3, T3 = 5, 6, 7
    E3, E4 = 8, 9
    P2, T4, D2 = 10, 11, 12

class MotorDriver:
    
    @classmethod
    def StopAllMotors(self, bus:smbus2.SMBus=None):
        for robotPinNumber in RobotPinNumbers:
            packet = Packet(opcode=Opcodes.stopPwm, payload=[robotPinNumber.value]) 
            print(packet)
            # checksum is calculated inside the Packet class

            # example of sending packet from rasberry to arduino via i2c
            if bus is not None:
                bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet)
            else:
                warnings.warn("smbus is not instanitiated")

    @classmethod
    def DisablePumps(self, bus:smbus2.SMBus=None):
        for pumpMotor in PumpMotors:
            packet = Packet(opcode=Opcodes.stopPwm, payload=[pumpMotor.value])
            print(packet)
            # checksum im calculated inside the Packet class

            # example of sending packet from rasberry to arduino via i2c
            if bus is not None:
                bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet)
            else:
                warnings.warn("smbus is not instanitiated")

    @classmethod        
    def MotorRun(self, motor:RobotPinNumbers, speed:int, bus:smbus2.SMBus=None):
        packet = Packet(opcode=Opcodes.startPwm, payload=[motor.value, speed])
        print(packet)
        # checksum im calculated inside the Packet class

        # example of sending packet from rasberry to arduino via i2c
        if bus is not None:
            bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet)
        else:
            warnings.warn("smbus is not instanitiated")
    
    @classmethod    
    def MotorStop(self, motor:RobotPinNumbers, bus:smbus2.SMBus=None):
        packet = Packet(opcode=Opcodes.stopPwm, payload=[motor.value])
        print(packet)
        # checksum im calculated inside the Packet class

        # example of sending packet from rasberry to arduino via i2c
        if bus is not None:
            bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet)
        else:
            warnings.warn("smbus is not instanitiated")

    @classmethod
    def GetMotorCurrent(self, motor:RobotPinNumbers, bus:smbus2.SMBus=None):
        packet = Packet(Opcodes.readAdc, payload=[motor.value])
        print(packet)
        # checksum im calculated inside the Packet class

        # example of sending packet from rasberry to arduino via i2c
        if bus is not None:
            bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet)
        else:
            warnings.warn("smbus is not instanitiated")

    @classmethod
    def getMotorFault(self, motor:RobotPinNumbers, bus:smbus2.SMBus=None):
        packet = Packet(Opcodes.setGpio, [motor.value])
        print(packet)
        # checksum im calculated inside the Packet class

        # example of sending packet from rasberry to arduino via i2c
        if bus is not None:
            bus.write_i2c_block_data(i2c_addr=ArduinoAddress.Arduino0, register=0x01, data=packet)
        else:
            warnings.warn("smbus is not instanitiated")
            
if __name__ == "__main__":
    # default_register = 0x1
    # bus = smbus2.SMBus(1)
    # bus.write_i2c_block_data()
    MotorDriver.DisablePumps()