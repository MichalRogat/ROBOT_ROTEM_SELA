from functions import GenericFunctions
from Entity import IMotor
import Entity

HIGH = 1
LOW = 0

class MotorDriver():
    # Class implements the MotorDriver from version 3.5 in a way that 
    # robot_main does not need any changes

    def __init__(self):
    # What are these for ?
        self.disable_motors = False
        # self.over_current = [False] * (RobotMotor.Pump3.value+1)
        # self.motor_speed = [0] * (RobotMotor.Pump3.value+1)
        # self.current_limit = [1300] * (RobotMotor.Pump3.value+1)
        # self.motor_current = [0] * (RobotMotor.Pump3.value+1)
    
    @classmethod
    def stopMotor(self, motor:IMotor):
        motor.stopMotor()

    @classmethod
    def motorRun(self, motor:IMotor, speed=0):
        motor.MotorRun(speed)

    @classmethod
    def StopAllMotors(self):
        for instance in IMotor.instances:
            instance.stopMotor()

        
motorDriver = MotorDriver()

# Example with driver
D1 = Entity.Driver(True, [12,11,10], 0)
motorDriver.motorRun(motor=D1, speed=50)

# Example with without driver
pumpMotor = Entity.Motor(pin=17) # A3
motorDriver.motorRun(motor=pumpMotor)

motorDriver.StopAllMotors()