# Each of 5 trailers has numerous motors (e.g D1, T1, E1)
# Each of the motors has 3 pins
# A3 is pump gpio P1 (digital write)
# A6 for readADC(full tank)
# A0-read motor 1 current sense
# A1-read motor 2 current sense
# A2-read motor 3 current sense

from abc import ABC, abstractmethod
from functions import GenericFunctions
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
    instances = []

    def __init__(self, I2CAddress):
        IMotor.instances.append(self)
        self.I2CAddress = I2CAddress
        self.dir = None

    @abstractmethod
    def stopMotor():
        pass

    @abstractmethod
    def MotorRun():
        pass

    @abstractmethod
    def get_a2d_mot_value():
        pass


class IIMU(ABC):
    instances_imu = []

    def __init__(self):
        IIMU.instances.append(self)

    @abstractmethod
    def trackAngle():
        pass




class Driver(IMotor):

    def __init__(self, I2CAddress, isUglyDriver, pins, checkOverCurrent):
        super().__init__(I2CAddress)
        self.isUglyDriver = isUglyDriver
        self.pins = pins
        self.checkOverCurrent = checkOverCurrent
        self.IN1type = None
        self.IN2type = None
        self.dir = None

    def stopMotor(self):
        self.gpio = LOW
        self.IN1 = 0
        self.IN2 = LOW
       
        GenericFunctions.callDriverFunction(self,retries=100)
        print(f"stopped motor: {self.I2CAddress} {pIdx[self.I2CAddress]['rcv']}")

    def MotorRun(self, speed):
        self.gpio = HIGH
        if speed >= 90:
            speed = 90
        if speed <= -90:
            speed = -90

        if self.isUglyDriver:
            # Move counterclock
            if speed >= 0:
                self.IN1 = speed
                self.IN2 = HIGH

                self.IN1type = STARTPWM
                self.IN2type = SETGPIO

            elif speed < 0:
                # Move cloclwise
                speed = abs(speed)
                self.IN1 = speed
                self.IN2 = LOW
                self.IN1type = STARTPWM
                self.IN2type = SETGPIO

        elif not self.isUglyDriver:
            # Move clockwise
            if speed >= 0:
                self.IN1 = HIGH
                self.IN2 = speed
                self.IN1type = SETGPIO
                self.IN2type = STARTPWM
            elif speed < 0:
                # Move counterclock
                speed = abs(speed)
                self.IN1 = speed
                self.IN2 = HIGH
                self.dir = 'b'
                self.IN1type = STARTPWM
                self.IN2type = SETGPIO

        self.gpio = HIGH
        # print("i am inheriting correctly")

        GenericFunctions.callDriverFunction(self)

    def get_a2d_mot_value(self):
        pass


class IMU(IIMU):

    def __init__(self, imu_address, Accel_Gyro_REG_L, Accel_Gyro_REG_H,
                 RegisterNum, value):
        super().__init__()
        IMU.instances_imu.append(self)
        self.imu_address = imu_address
        self.Accel_Gyro_REG_L = Accel_Gyro_REG_L
        self.Accel_Gyro_REG_H = Accel_Gyro_REG_H
        self.RegisterNum = RegisterNum
        self.value = value

class ITrailer(ABC):
    trailer_instances = []

    def __init__(self):
        super().__init__()
        ITrailer.trailer_instances.append(self)


class Trailer1(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.I2CAddress = I2CAddress
        self.name = '1'
        self.driver1 = Driver(I2CAddress=I2CAddress,
                                   isUglyDriver=False,
                                   pins=[9, 6, 7],
                                   checkOverCurrent=14)
        self.turn1 = Driver(I2CAddress=I2CAddress,
                            isUglyDriver=False,
                            pins=[3, 5, 2],
                            checkOverCurrent=15)
        self.pump1 = Driver(I2CAddress=I2CAddress, isUglyDriver=False, pins=[11,10,12], checkOverCurrent=16)

class Trailer2(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.name = '2'
        self.I2CAddress = I2CAddress
        self.elevation1 = Driver(I2CAddress=I2CAddress,
                         isUglyDriver=False,
                         pins=[3, 5, 2],
                         checkOverCurrent=14)
        self.elevation2 = Driver(I2CAddress=I2CAddress,
                         isUglyDriver=False,
                         pins=[9, 6, 7],
                         checkOverCurrent=15)


class Trailer3(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.name = '3'
        self.I2CAddress = I2CAddress
        self.turn2 = Driver(I2CAddress=I2CAddress,
                         isUglyDriver=False,
                         pins=[3, 5, 2],
                         checkOverCurrent=14)
        self.turn3 = Driver(I2CAddress=I2CAddress,
                         isUglyDriver=False,
                         pins=[9, 6, 7],
                         checkOverCurrent=15)


class Trailer4(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.name = '4'
        self.I2CAddress = I2CAddress
        self.elevation3 = Driver(I2CAddress=I2CAddress,
                         isUglyDriver=False,
                         pins=[9, 6, 7],
                         checkOverCurrent=14)
        self.elevation4 = Driver(I2CAddress=I2CAddress,
                         isUglyDriver=False,
                         pins=[3, 5, 2],
                         checkOverCurrent=15)


class Trailer5(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.name = '5'
        self.I2CAddress = I2CAddress
        self.driver2 = Driver(I2CAddress=I2CAddress,
                              isUglyDriver=False,
                              pins=[3, 5, 2],
                              checkOverCurrent=14)
        self.turn4 = Driver(I2CAddress=I2CAddress,
                         isUglyDriver=False,
                         pins=[9, 6, 7],
                         checkOverCurrent=15)
        self.pump2 = Driver(I2CAddress=I2CAddress, isUglyDriver=False, pins=[11,10,12], checkOverCurrent=16)
