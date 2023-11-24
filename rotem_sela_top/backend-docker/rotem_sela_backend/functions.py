from smbus2 import SMBus
import time
from threading import Lock
import traceback

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
    def __init__(self,  opcode, pIdx, payload:list):
        self.SOT = 0xAA
        self.opcode = opcode
        self.packetIdx = pIdx;
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
            checksum += data
        return checksum%255
    
    def to_array(self):
        return [self.SOT, self.packetIdx,  self.opcode, self.payload_length] + self.payload + [self.checksum]

    def __repr__(self) -> str:
        return f"{self.SOT}, {self.opcode}, {self.packetIdx}, {self.payload_length}, {self.payload}, {self.checksum}" 

pIdx = {}
def i2c_write(i2c_addr, data, retries=10):
    initRetries = retries
    
    global pIdx
    if i2c_addr not in pIdx:
        pIdx[i2c_addr] = {'rcv':-1, 'send':0}
    pIdx[i2c_addr]['ack'] = False
    while retries > 0:
        try:
            lock.acquire()
            bus.write_i2c_block_data(i2c_addr=i2c_addr, register=0x1, data=data)
            break
            
        except Exception as e:
            retries = retries - 1
            time.sleep(0.02)
            continue
        finally:
            lock.release()
        
        # if i2c_addr == 0x11:
                
        #     time.sleep(0.02)
        #     if pIdx[i2c_addr]['ack']:
        #         break
        # else:
        #     break
    
    if retries <= 0:
        print("Failed i2c "+str(i2c_addr))
        pass
    return

def i2c_read(i2c_addr, len):
    res = None
    
    lock.acquire()
    
    try:
        res = bus.read_i2c_block_data(i2c_addr=i2c_addr, register=0x1, length=len)
    finally:
        lock.release()
    
    return res

def sendPacketOrDebug(packet:Packet, i2c_addr, retries=10):
    if DEBUG:
        print(f"packet: {packet}")
    else:
        # print(f"i2caddr: {i2c_addr}, packet:{packet}")
        i2c_write(i2c_addr, packet.to_array(), retries=retries)
       

class GenericFunctions:

    @classmethod
    def callDriverFunction(cls, driver, retries=10):
        # Controls generic driver which controls over 3 pins.
        # func - the function to perform e.g. stopMotor startMotor
        # motor - the motor to stop
        global pIdx
        if driver.I2CAddress not in pIdx:
            pIdx[driver.I2CAddress] = {'rcv':-1, 'send':0}

        packet2 = Packet(SETGPIO, pIdx[driver.I2CAddress]['send'], [driver.pins[SLEEP], driver.gpio])
        sendPacketOrDebug(packet2, driver.I2CAddress, retries=retries)
        if driver.IN1type is None:
            return
        
        if driver.IN1type == STARTPWM:
            packet4 = Packet(driver.IN2type, pIdx[driver.I2CAddress]['send'], [driver.pins[IN2], driver.IN2])
            sendPacketOrDebug(packet4,  driver.I2CAddress, retries=retries)
            packet3 = Packet(driver.IN1type, pIdx[driver.I2CAddress]['send'], [driver.pins[IN1], driver.IN1])
            sendPacketOrDebug(packet3, driver.I2CAddress, retries=retries)
        else:
            packet3 = Packet(driver.IN1type, pIdx[driver.I2CAddress]['send'], [driver.pins[IN1], driver.IN1])
            sendPacketOrDebug(packet3, driver.I2CAddress, retries=retries)
            packet4 = Packet(driver.IN2type, pIdx[driver.I2CAddress]['send'], [driver.pins[IN2], driver.IN2])
            sendPacketOrDebug(packet4, driver.I2CAddress, retries=retries)

    @classmethod
    def callDigitalGpioFunction(cls, motor):
        global pIdx
        if motor.I2CAddress not in pIdx:
            pIdx[motor.I2CAddress] = {'rcv':-1, 'send':0}
        # This method controls generic digital gpio
        packet = Packet(SETGPIO, pIdx[motor.I2CAddress]['send'], payload=[motor.pin, motor.gpio])
        sendPacketOrDebug(packet, motor.I2CAddress)
        # print(packet, motor.I2CAddress)
    @classmethod
    def setGpioFunction(cls, motor, pin, on):
        global pIdx
        if motor.I2CAddress not in pIdx:
            pIdx[motor.I2CAddress] = {'rcv':-1, 'send':0}
        # This method controls generic digital gpio
        packet = Packet(SETGPIO, pIdx[motor.I2CAddress]['send'], payload=[pin, on])
        sendPacketOrDebug(packet, motor.I2CAddress)
        # print(packet, motor.I2CAddress)
    @classmethod
    def callReadNano(cls, trailers, nanoTelemetry):
        for trailer in trailers:
            try:
                global pIdx
                if trailer.I2CAddress not in pIdx:
                    pIdx[trailer.I2CAddress] = {'rcv':-1, 'send':0}
                ret_byte = i2c_read(trailer.I2CAddress, 32)

                if ret_byte[0] == 170 and ret_byte[4] == 175:
                    print("Ack received "+str(trailer.name))
                    pIdx[trailer.I2CAddress]['ack'] = True
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
            except Exception as e:
                # print(f"Trailer {trailer.name}")
                # traceback.print_exc()
                pass
            finally:
                time.sleep(0.01)
        # print(nanoTelemetry)
        
if __name__ == "__main__":
    #packet = Packet(SETGPIO, payload=[17, HIGH])
    #sendPacketOrDebug(packet, 0x13)
    pass
