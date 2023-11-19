from Entity import IMotor
import Entity

HIGH = 1
LOW = 0

class MotorDriver():
    # Class implements the MotorDriver from version 3.5 in a way that 
    # robot_main does not need any changes

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
        motor.MotorRun(speed)

    @classmethod
    def StopAllMotors(self):
        print("Disable Motors")
        if not MotorDriver.disable_motors:
            for instance in IMotor.instances:
                instance.stopMotor()

        self.disable_motors = False

    @classmethod
    def DisablePumps(self):
        print("Disable Pumps")
        for pumpInstacne in Entity.Pump.instances:
            pumpInstacne.stopMotor()

    @classmethod
    def get_a2d_mot_value(self, motor:Entity.IMotor):
        motor.get_a2d_mot_value()

if __name__ == "__main__":
    # unit test MotorDriver.py, Entity.py, Functions.py

    motorDriver = MotorDriver()

    # Example with driver
    D1 = Entity.Driver(True, [12,11,10], 0)
    # motorDriver.MotorRun(motor=D1, speed=50)

    # Example with without driver
    # pumpMotor = Entity.Pump(pin=17) # A3
    # motorDriver.MotorRun(motor=pumpMotor)

    motorDriver.StopAllMotors()
    # motorDriver.DisablePumps()