# Each of 5 trailers has numerous motors (e.g D1, T1, E1)
# Each of the motors has 3 pins
# A3 is pump gpio P1 (digital write)
# A6 for readADC(full tank)
# A0-read motor1 current sense
# A1-read motor 2 current sense
# A2-read motor 3 current sense 
from abc import ABC

class ITrailer(ABC):
    I2CAddress = 0
    checkFullTankPin = 0
    activatePumpPin = 0
    readBatteryPin = 0
    readIMUPin = 0

class IMotor(ITrailer):
    isUglyFlag = False
    pins = []
    chekOverCurrent = 0

class Trailer1():
    I2CAddress = 0x1
    checkFullTankPin = 20 #A6 - readADC
    activatePumpPin = 17 #A3 - digitalOutput (High or Low)
    readBatteryPin = None 
    IMU_SCL_pin = 8 # D8 on nano - Software I2C
    IMU_SDA_pin = 4 # D4 on nano - Software I2C

class D1Motor(Trailer1, IMotor):
    isUglyDriver = True
    pins = [2, 5, 3]
    checkOverCurrent = 14 #A0 - readADC

class T1Motor(Trailer1):
    isUglyDriver = False
    pins = [7, 9, 6]
    checkOverCurrent = 15 #A1 - readADC

class Trailer2():

    class E1():
        isUglyDriver = True

myMotor = D1Motor()
print(myMotor.pins)