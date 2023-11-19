# Each of 5 trailers has numerous motors (e.g D1, T1, E1)
# Each of the motors has 3 pins
# A3 is pump gpio P1 (digital write)
# A6 for readADC(full tank)
# A0-read motor1 current sense
# A1-read motor 2 current sense
# A2-read motor 3 current sense 

from abc import ABC, abstractmethod
from functions import GenericFunctions
import MotorDriver

HIGH = 1
LOW = 0

class ITrailer(ABC):
    I2CAddress = 0
    checkFullTankPin = 0    # - readADC
    pumpPin = 0     # - digital output (High or Low)
    readBatteryPin = 0      # - readADC
    readIMUPin = 0          # - software I2C

class IMotor(ITrailer):
    isUglyDriver = False
    pins = []
    chekOverCurrent = 0

    @abstractmethod
    def stopMotor():
        pass

    @abstractmethod
    def MotorRun():
        pass


class Trailer1():
    I2CAddress = 0x1
    checkFullTankPin = 20 #A6 - readADC
    activatePumpPin = 17 #A3 - digitalOutput (High or Low)
    readBatteryPin = None 
    IMU_SCL_pin = 8 # D8 on nano - Software I2C
    IMU_SDA_pin = 4 # D4 on nano - Software I2C

class Driver(IMotor):
        
    def __init__(self, isUglyDriver, pins, checkOverCurrent):
        self.isUglyDriver = isUglyDriver
        self.pins = pins
        self.checkOverCurrent = checkOverCurrent

    def stopMotor(self):
        self.gpio = LOW
        self.pwm = 0
        self.extra = LOW
        GenericFunctions.callDriverFunction(self)

    def MotorRun(self, speed):

        if speed >= 90:
            speed = 90
        if speed <= -90:
            speed = -90

        if self.isUglyDriver:
            # Move counterclock
            if speed >= 0:
                self.pwm = speed
                self.extra = HIGH
            elif speed < 0:
            # Move cloclwise
                speed = abs(speed)
                self.pwm = speed
                self.extra = LOW

        elif not self.isUglyDriver:
            # Move clockwise
            if speed >= 0:
                self.pwm = HIGH
                self.extra = speed
            elif speed < 0:
            # Move counterclock
                speed = abs(speed)
                self.pwm = speed
                self.extra = HIGH
        
        self.gpio = HIGH

        GenericFunctions.callDriverFunction(self)


class Motor(IMotor):
    
    def __init__(self, pin):
        self.pin = pin

    def MotorRun(self, speed):
        self.gpio = HIGH
        GenericFunctions.callDigitalGpioFunction(self)

    def stopMotor(self):
        self.gpio = LOW
        GenericFunctions.callDigitalGpioFunction(self)

# D1 = Driver(True, [12, 11, 10], 0)
# # D1.

# pumpMotor = Motor(pin=17, gpio=LOW) # A3
# pumpMotor.startMotor()