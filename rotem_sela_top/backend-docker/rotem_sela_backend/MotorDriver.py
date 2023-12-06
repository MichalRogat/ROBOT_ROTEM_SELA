from Entity import IMotor
import Entity
import Entity as Entity
import time

HIGH = 1
LOW = 0

class MotorDriver():

    def __init__(self):
        self.trailer1 = Entity.Trailer1(I2CAddress = 0x11)
        self.trailer2 = Entity.Trailer2(I2CAddress = 0x22)
        self.trailer3 = Entity.Trailer3(I2CAddress = 0x33)
        self.trailer4 = Entity.Trailer4(I2CAddress = 0x44)
        self.trailer5 = Entity.Trailer5(I2CAddress = 0x55)

    disable_motors = False
    
    @classmethod
    def stopMotor(self, motor:IMotor):
        motor.stopMotor()

    @classmethod
    def MotorRun(self, motor:IMotor, speed=0):
        motor.MotorRun(speed)
        
    @classmethod
    def StopAllMotors(self):
        print("Disable Motors")
        # if not MotorDriver.disable_motors:
        for instance in IMotor.motor_instances:
           
            instance.stopMotor()
        self.disable_motors = False

    @classmethod
    def DisablePumps(self):
        print("Disable Pumps")
        for pumpInstacne in Entity.Pump.instances:
            pumpInstacne.stopMotor()

