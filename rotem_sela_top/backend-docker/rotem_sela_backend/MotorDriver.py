from Entity import IMotor
import Entity
import threading
from functions import GenericFunctions
import Entity as Entity
import time

HIGH = 1
LOW = 0

class MotorDriver():
# 
    def __init__(self):
        trailer1 = Entity.Trailer1(I2CAddress = 0x11)
        trailer2 = Entity.Trailer2(I2CAddress = 0x12)
        trailer3 = Entity.Trailer3(I2CAddress = 0x13)
        trailer4 = Entity.Trailer4(I2CAddress = 0x14)
        trailer5 = Entity.Trailer5(I2CAddress = 0x15)

        def init_thread():
                GenericFunctions.callReadNano(Entity.ITrailer.trailer_instances)
        threading.Thread(target=init_thread).start()


    disable_motors = False
    # over_current = [False] * (RobotMotor.Pump3.value+1)
    # motor_speed = [0] * (RobotMotor.Pump3.value+1)
    # current_limit = [1300] * (RobotMotor.Pump3.value+1)
    # motor_current = [0] * (RobotMotor.Pump3.value+1)
    
    @classmethod
    def stopMotor(self, motor:IMotor):
        motor.stopMotor()

    @classmethod
    def MotorRun(self, motor:IMotor, speed=0):
        if not motor.isUglyDriver:
            motor.MotorRun(100-abs(speed) if speed > 0 else -(100-abs(speed)))
        else:
            motor.MotorRun(speed)

    @classmethod
    def StopAllMotors(self):
        print("Disable Motors")
        # if not MotorDriver.disable_motors:
        for instance in IMotor.instances:
            instance.gpio = LOW
            instance.stopMotor()
        self.disable_motors = False

    @classmethod
    def DisablePumps(self):
        print("Disable Pumps")
        for pumpInstacne in Entity.Pump.instances:
            pumpInstacne.stopMotor()

motorDriver = MotorDriver()