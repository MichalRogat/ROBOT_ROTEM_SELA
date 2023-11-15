from enum import Enum

class RobotMotor(Enum):
    P1, 


class MotorDriver:
    def __init__(self):
        return NotImplementedError

    def StopAllMotors(self):
        return NotImplementedError

    def DisableMuxPWMC(self):
        return NotImplementedError
        
    def DisablePumps(self):
        return NotImplementedError
        
    def MotorRun(self, motor, speed):
        return NotImplementedError

    def MotorStop(self, motor):
        return NotImplementedError

    def get_a2d_mot_value(self, motor, a2d_values):
       return NotImplementedError
    
    def MotorTestCurrentOverload(self, a2d_values):
        return NotImplementedError

    def OverCurrentDueTime(self,motor):
        return NotImplementedError

if __name__ == "__main__":
    pass