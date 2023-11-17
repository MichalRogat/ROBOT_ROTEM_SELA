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


class Driver():
        
    def __init__(self, isUglyDriver, pins, checkOverCurrent):
        self.isUglyDriver = isUglyDriver
        self.pins = pins
        self.checkOverCurrent = checkOverCurrent

    def stopDriver(self):
        self.gpio = LOW
        self.pwm = 0
        self.extra = LOW
        GenericFunctions.callDriverFunction(self)

class Motor():
    
    def __init__(self, pin, gpio):
        self.pin = pin
        self.gpio = gpio

    def startMotor(self):
        GenericFunctions.callDigitalGpioFunction(self)


pumpMotor = Motor(pin=17, gpio=LOW) # A3
pumpMotor.startMotor()