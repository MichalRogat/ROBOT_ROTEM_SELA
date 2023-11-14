from pwm_rpi import RobotMotor,MotorDriver
import time
def CheckBacklashTimer():
    return

if __name__=="__main__":
    Motor = MotorDriver()
    Motor.MotorRun(RobotMotor.Drive1, 'forward', 100)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.Drive1)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.Drive1, 'reverse', 100)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.Drive1)
    
