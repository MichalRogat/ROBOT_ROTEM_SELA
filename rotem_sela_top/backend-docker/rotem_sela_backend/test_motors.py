from MotorDriver import MotorDriver
from Entity import Trailer1, Trailer2, Trailer3, Trailer4, Trailer5

trailer1 = Trailer1(I2CAddress = 0x11)

# MotorDriver.MotorRun(trailer1.driver1, speed=90)
# MotorDriver.stopMotor(trailer1.driver1)
# MotorDriver.MotorRun(trailer1.driver1, speed=-90)
# MotorDriver.stopMotor(trailer1.driver1)
# MotorDriver.MotorRun(trailer1.driver1, speed=10)
# MotorDriver.stopMotor(trailer1.driver1)
# MotorDriver.MotorRun(trailer1.driver1, speed=-10)
# MotorDriver.stopMotor(trailer1.driver1)

# MotorDriver.MotorRun(trailer1.turn1, 90)
# MotorDriver.stopMotor(trailer1.turn1)
# MotorDriver.MotorRun(trailer1.turn1, -90)
# MotorDriver.stopMotor(trailer1.turn1)
# MotorDriver.MotorRun(trailer1.turn1, 10)
# MotorDriver.stopMotor(trailer1.turn1)
# MotorDriver.MotorRun(trailer1.turn1, -10)
# MotorDriver.stopMotor(trailer1.turn1)

MotorDriver.MotorRun(trailer1.pump1, 90)
MotorDriver.stopMotor(trailer1.pump1)

# trailer2 = Trailer2(I2CAddress = 0x12)
# MotorDriver.MotorRun(trailer2.elevation1, speed=90)
# MotorDriver.stopMotor(trailer2.elevation1)
# MotorDriver.MotorRun(trailer2.elevation1, speed=-90)
# MotorDriver.stopMotor(trailer2.elevation1)

# MotorDriver.MotorRun(trailer2.elevation2, speed=90)
# MotorDriver.stopMotor(trailer2.elevation2)
# MotorDriver.MotorRun(trailer2.elevation2, speed=-90)
# MotorDriver.stopMotor(trailer2.elevation2)

# trailer3 = Trailer3(I2CAddress = 0x13)
# MotorDriver.MotorRun(trailer3.turn2, speed=90)
# MotorDriver.stopMotor(trailer3.turn2)
# MotorDriver.MotorRun(trailer3.turn2, speed=-90)
# MotorDriver.stopMotor(trailer3.turn2)

# MotorDriver.MotorRun(trailer3.turn3, speed=90) # not working
# MotorDriver.stopMotor(trailer3.turn3)
# MotorDriver.MotorRun(trailer3.turn3, speed=-90) # not working
# MotorDriver.stopMotor(trailer3.turn3)

# trailer4 = Trailer4(I2CAddress = 0x14)
# MotorDriver.MotorRun(trailer4.elevation3, speed=90) # not strong enogh to move, not moving
# MotorDriver.stopMotor(trailer4.elevation3)
# MotorDriver.MotorRun(trailer4.elevation3, speed=-90) # not strong enogh to move, not moving
# MotorDriver.stopMotor(trailer4.elevation3)
# MotorDriver.MotorRun(trailer4.elevation4, speed=90) # not strong enough to move, not moving
# MotorDriver.stopMotor(trailer4.elevation4)
# MotorDriver.MotorRun(trailer4.elevation4, speed=-90) # not strong enough to move, not moving
# MotorDriver.stopMotor(trailer4.elevation4)

trailer5 = Trailer5(I2CAddress = 0x15)
# MotorDriver.MotorRun(trailer5.driver2, speed=90)
# MotorDriver.stopMotor(trailer5.driver2)
# MotorDriver.MotorRun(trailer5.driver2, speed=-90)
# MotorDriver.stopMotor(trailer5.driver2)
# MotorDriver.MotorRun(trailer5.driver2, speed=10)
# MotorDriver.stopMotor(trailer5.driver2)
# MotorDriver.MotorRun(trailer5.driver2, speed=-10)   
# MotorDriver.stopMotor(trailer5.driver2)

# MotorDriver.MotorRun(trailer5.turn4, 30) 
# MotorDriver.stopMotor(trailer5.turn4)
# MotorDriver.MotorRun(trailer5.turn4, -30) 
# MotorDriver.stopMotor(trailer5.turn4)
MotorDriver.MotorRun(trailer5.pump2, 90)
MotorDriver.stopMotor(trailer5.pump2)

MotorDriver.StopAllMotors()