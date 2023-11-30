# Each of 5 trailers has numerous motors (e.g D1, T1, E1)
# Each of the motors has 3 pins
# A3 is pump gpio P1 (digital write)
# A6 for readADC(full tank)
# A0-read motor 1 current sense
# A1-read motor 2 current sense
# A2-read motor 3 current sense

from abc import ABC, abstractmethod
from functions import Packet
from functions import pIdx


HIGH = 1
LOW = 0

IN1 = 0
IN2 = 1
SLEEP = 2

STARTPWM = 0
STOPPWM = 1
SETGPIO = 2
READADC = 3
GETGPIO = 4


class IMotor(ABC):
    motor_instances = []

    def __init__(self, I2CAddress):
        super().__init__()
        IMotor.motor_instances.append(self)
        self.I2CAddress = I2CAddress
        self.speed = 0
       
    @abstractmethod
    def stopMotor():
        pass

    @abstractmethod
    def MotorRun():
        pass

class Driver(IMotor):

    def __init__(self, I2CAddress, motorNum):
        super().__init__(I2CAddress)
        self.motorNum = motorNum
        
    def stopMotor(self):

        self.speed = 0

    def MotorRun(self, speed):
       
        if speed >= 90:
            speed = 90
        if speed <= -90:
            speed = -90

        self.speed = speed

class ITrailer(ABC):
    trailer_instances = []
    gpio = {}
    def __init__(self):
        super().__init__()
        ITrailer.trailer_instances.append(self)
    
    def addGpio(self, pin, val):
        self.gpio[pin] = val;

    def GetGpioState(self):
        res = []
        for pin in self.gpio:
            res.append(pin)
            res.append(self.gpio[pin])
        return res
    
    @abstractmethod
    def GetState():
        pass

class Trailer1(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.I2CAddress = I2CAddress
        self.name = '1'
        self.driver1 = Driver(I2CAddress,2)
        self.turn1 = Driver(I2CAddress,1)
        self.pump1 = Driver(I2CAddress, 3)
    
    def GetState(self):
        return Packet([self.turn1.speed, self.driver1.speed, self.pump1.speed]+self.GetGpioState(), 0x11)

class Trailer2(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.name = '2'
        self.I2CAddress = I2CAddress
        self.elevation1 = Driver(I2CAddress,1)
        self.elevation2 = Driver(I2CAddress,2)

    def GetState(self):
        return Packet([self.elevation1.speed, self.elevation2.speed, 0]+self.GetGpioState(), 0x22)

class Trailer3(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.name = '3'
        self.I2CAddress = I2CAddress
        self.turn2 = Driver(I2CAddress,1)
        self.turn3 = Driver(I2CAddress,2)

    def GetState(self):
        return Packet([self.turn2.speed, self.turn3.speed, 0]+self.GetGpioState(), 0x33)

class Trailer4(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.name = '4'
        self.I2CAddress = I2CAddress
        self.elevation3 = Driver(I2CAddress,2)
        self.elevation4 = Driver(I2CAddress,1)
    def GetState(self):
        return Packet([self.elevation4.speed, self.elevation3.speed, 0]+self.GetGpioState(), 0x44)

class Trailer5(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.name = '5'
        self.I2CAddress = I2CAddress
        self.driver2 = Driver(I2CAddress,1)
        self.turn4 = Driver(I2CAddress,2)
        self.pump2 = Driver(I2CAddress,3)

    def GetState(self):
        return Packet([self.driver2.speed, self.turn4.speed, self.pump2.speed]+self.GetGpioState(), 0x55)