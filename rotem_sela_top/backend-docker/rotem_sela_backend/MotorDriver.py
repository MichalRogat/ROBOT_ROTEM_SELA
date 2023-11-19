from functions import GenericFunctions
import Entity
from Entity import IMotor

HIGH = 1
LOW = 0

class MotorDriver():
    # Class implements the MotorDriver from version 3.5 in a way that 
    # robot_main does not need any changes
    
    @classmethod
    def stopMotor(self, motor:IMotor):
        motor.stopMotor()

    def motorRun(self, motor:IMotor, speed=0):
        motor.MotorRun(speed)
        
# Example with driver
motorDriver = MotorDriver()
D1 = Entity.Driver(True, [12,11,10], 0)
motorDriver.motorRun(motor=D1, speed=50)
motorDriver.stopMotor(D1)

# Example with without driver
pumpMotor = Entity.Motor(pin=17) # A3
motorDriver.motorRun(motor=pumpMotor)
motorDriver.stopMotor(motor=pumpMotor)