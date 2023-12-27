from abc import ABC, abstractmethod
from functions import Packet

import json

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

# with open('../../../../entitiesFlipping.json', 'r') as file:
#         json_data = json.load(file)
#         trailer1_list = json_data.get("trailer1", [])
#         trailer2_list = json_data.get("trailer2", [])
#         trailer3_list = json_data.get("trailer3", [])
#         trailer4_list = json_data.get("trailer4", [])
#         trailer5_list = json_data.get("trailer5", [])


class IMotor(ABC):
    motor_instances = []

    def __init__(self, I2CAddress, name):
        super().__init__()
        IMotor.motor_instances.append(self)
        self.I2CAddress = I2CAddress
        self.speed = 0
        self.name = name
       
    @abstractmethod
    def stopMotor():
        pass

    @abstractmethod
    def MotorRun():
        pass

    def getByName(name):
        for motor in IMotor.motor_instances:
            if motor.name == name:
                return motor

class Driver(IMotor):

    def __init__(self, I2CAddress, motorNum, name):
        super().__init__(I2CAddress, name)
        self.motorNum = motorNum
        self.name = name
        
    def stopMotor(self):
        print(f"stopped motor {self.name}")
        self.speed = 0

    def MotorRun(self, speed):
        if speed >= 90:
            speed = 90
        if speed <= -90:
            speed = -90        
        self.speed = speed
        print(f"motor run {self.name} {self.speed}")


class ITrailer(ABC):
    trailer_instances = []
    gpio = {}
    def __init__(self):
        super().__init__()
        ITrailer.trailer_instances.append(self)
    
    def addGpio(self, pin, val):
        self.gpio[pin] = val

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
        self.driver_array = [{}, {}, {}]

        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
            trailer1_list = json_data.get("trailer1", [])

        if "driver1" in trailer1_list:
            driver1_index = trailer1_list.index("driver1")
            self.driver1 = Driver(I2CAddress, driver1_index+1, "driver1")
            self.driver_array[driver1_index] = self.driver1
            
        if "turn1" in trailer1_list:
            turn1_index = trailer1_list.index("turn1")
            self.turn1 = Driver(I2CAddress, turn1_index+1, "turn1")
            self.driver_array[turn1_index] = self.turn1

        if "pump1" in trailer1_list:
            pump1_index = trailer1_list.index("pump1")
            self.pump1 = Driver(I2CAddress,pump1_index+1, "pump1")
            self.driver_array[pump1_index] = self.pump1

    def GetState(self):
        return Packet([self.driver_array[0].speed, self.driver_array[1].speed, self.driver_array[2].speed]+self.GetGpioState(), pIdx=self.I2CAddress)

class Trailer2(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.I2CAddress = I2CAddress
        self.name = '2'
        self.driver_array = [{}, {}, {}]

        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
            trailer2_list = json_data.get("trailer2", [])

        if "elevation1" in trailer2_list:
            elevation1_index = trailer2_list.index("elevation1")
            self.elevation1 = Driver(I2CAddress, elevation1_index+1, "elevation1")
            self.driver_array[elevation1_index] = self.elevation1
            
        if "elevation2" in trailer2_list:
            elevation2_index = trailer2_list.index("elevation2")
            self.elevation2 = Driver(I2CAddress, elevation2_index+1, "elevation2")
            self.driver_array[elevation2_index] = self.elevation2

    def GetState(self):
        return Packet([self.driver_array[0].speed, self.driver_array[1].speed, 0]+self.GetGpioState(), pIdx=self.I2CAddress)

class Trailer3(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.I2CAddress = I2CAddress
        self.name = '3'
        self.driver_array = [{}, {}, {}]

        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
            trailer3_list = json_data.get("trailer3", [])

        if "turn2" in trailer3_list:
            turn2_index = trailer3_list.index("turn2")
            self.turn2 = Driver(I2CAddress, turn2_index+1, "turn2")
            self.driver_array[turn2_index] = self.turn2
            
        if "turn3" in trailer3_list:
            turn3_index = trailer3_list.index("turn3")
            self.turn3 = Driver(I2CAddress, turn3_index+1, "turn3")
            self.driver_array[turn3_index] = self.turn3

        if "cooler" in trailer3_list:
            cooler_index = trailer3_list.index("cooler")
            self.cooler = Driver(I2CAddress, cooler_index+1, "cooler")
            self.driver_array[cooler_index] = self.cooler

    def GetState(self):
        return Packet([self.driver_array[0].speed, self.driver_array[1].speed, self.driver_array[2].speed]+self.GetGpioState(), pIdx=self.I2CAddress)

class Trailer4(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.I2CAddress = I2CAddress
        self.name = '4'
        self.driver_array = [{}, {}, {}]

        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
            trailer4_list = json_data.get("trailer4", [])

        if "elevation4" in trailer4_list:
            elevation4_index = trailer4_list.index("elevation4")
            self.elevation4 = Driver(I2CAddress, elevation4_index+1, "elevation4")
            self.driver_array[elevation4_index] = self.elevation4

        if "elevation3" in trailer4_list:
            elevation3_index = trailer4_list.index("elevation3")
            self.elevation3 = Driver(I2CAddress, elevation4_index+1, "elevation3")
            self.driver_array[elevation3_index] = self.elevation3

    def GetState(self):
        return Packet([self.driver_array[0].speed, self.driver_array[1].speed, 0]+self.GetGpioState(), pIdx=self.I2CAddress)

class Trailer5(ITrailer):

    def __init__(self, I2CAddress):
        super().__init__()
        self.I2CAddress = I2CAddress
        self.name = '5'

        self.driver_array = [{}, {}, {}]

        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
            trailer5_list = json_data.get("trailer5", [])

        if "driver2" in trailer5_list:
            driver2_index = trailer5_list.index("driver2")
            self.driver2 = Driver(I2CAddress, driver2_index+1, "driver2")
            self.driver_array[driver2_index] = self.driver2

        if "turn4" in trailer5_list:
            turn4_index = trailer5_list.index("turn4")
            self.turn4 = Driver(I2CAddress, turn4_index+1, "turn4")
            self.driver_array[turn4_index] = self.turn4

        if "pump2" in trailer5_list:
            pump2_index = trailer5_list.index("pump2")
            self.pump2 = Driver(I2CAddress, pump2_index+1, "pump2")
            self.driver_array[pump2_index] = self.pump2

    def GetState(self):
        return Packet([self.driver_array[0].speed, self.driver_array[1].speed, self.driver_array[2].speed]+self.GetGpioState(), pIdx=self.I2CAddress)
