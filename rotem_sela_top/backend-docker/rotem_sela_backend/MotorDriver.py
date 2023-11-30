from Entity import IMotor
import Entity
import Entity as Entity
import time

HIGH = 1
LOW = 0

class MotorDriver():
# 
    def __init__(self):
        self.trailer1 = Entity.Trailer1(I2CAddress = 0x11)
        self.trailer2 = Entity.Trailer2(I2CAddress = 0x22)
        self.trailer3 = Entity.Trailer3(I2CAddress = 0x33)
        self.trailer4 = Entity.Trailer4(I2CAddress = 0x44)
        self.trailer5 = Entity.Trailer5(I2CAddress = 0x55)

    disable_motors = False
    # over_current = [False] * (RobotMotor.Pump3.value+1)
    # motor_speed = [0] * (RobotMotor.Pump3.value+1)
    # current_limit = [1300] * (RobotMotor.Pump3.value+1)
    # motor_current = [0] * (RobotMotor.Pump3.value+1)
    
    @classmethod
    def stopMotor(self, motor:IMotor):
        print(f"stopping motor {motor}")
        motor.stopMotor()

    @classmethod
    def MotorRun(self, motor:IMotor, speed=0):
        print(f"running motor {motor} at speed {speed}")
        motor.MotorRun(speed)
        
    @classmethod
    def StopAllMotors(self):
        print("Disable Motors")
        # if not MotorDriver.disable_motors:
        for instance in IMotor.instances:
           
            instance.stopMotor()
        self.disable_motors = False

    @classmethod
    def DisablePumps(self):
        print("Disable Pumps")
        for pumpInstacne in Entity.Pump.instances:
            pumpInstacne.stopMotor()

