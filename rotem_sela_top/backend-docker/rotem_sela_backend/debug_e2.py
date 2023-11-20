from Entity import Trailer2, Driver
from MotorDriver import MotorDriver

# trailer2 = Trailer2(I2CAddress = 0x12)
# MotorDriver.MotorRun(trailer2.e2, speed=90) # not moving, still not moving
# MotorDriver.stopMotor(trailer2.e2)

e2 = Driver(I2CAddress=0x12,
                    isUglyDriver=False,
                    pins=[9, 6, 7],
                    checkOverCurrent=15)
e2.MotorRun(10)