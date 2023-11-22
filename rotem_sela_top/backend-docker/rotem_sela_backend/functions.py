from smbus2 import SMBus
import time
from threading import Lock

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
        # print(packet, motor.I2CAddress)

    @classmethod
    def callReadNano(cls, trailers, nanoTelemetry):
        for trailer in trailers:
            try:
                ret_byte = i2c_read(trailer.I2CAddress, 32)

                nanoTelemetry['FullTank'+trailer.name] = int.from_bytes(ret_byte[0:2], byteorder='little')
                nanoTelemetry['drive'+trailer.name] = int.from_bytes(ret_byte[2:4], byteorder='little')
                nanoTelemetry['m2CS'] = int.from_bytes(ret_byte[4:6], byteorder='little')
                nanoTelemetry['m3CS'] = int.from_bytes(ret_byte[6:8], byteorder='little')
                nanoTelemetry['batteryRead'] = int.from_bytes(ret_byte[8:10], byteorder='little')
                nanoTelemetry['fault1'] = int.from_bytes(ret_byte[10:11], byteorder='little')
                nanoTelemetry['fault2'] = int.from_bytes(ret_byte[11:12], byteorder='little')
                nanoTelemetry['fault3'] = int.from_bytes(ret_byte[12:13], byteorder='little')
                print("->"+str(ret_byte[13]))
                print(ret_byte[14])
                print(ret_byte[15])
                print(ret_byte[16])

                nanoTelemetry['imu'+trailer.name] = [int.from_bytes(ret_byte[13:17], byteorder='little', signed=True)/10.0,
                                                     int.from_bytes(ret_byte[17:21], byteorder='little', signed=True)/10.0,
                                                     int.from_bytes(ret_byte[21:25], byteorder='little', signed=True)/10.0
                                                    ]
            except Exception as e:
                print(f"Trailer {trailer.name} {e}")
            finally:
                time.sleep(0.05)
        print(nanoTelemetry)
        
if __name__ == "__main__":
    pass