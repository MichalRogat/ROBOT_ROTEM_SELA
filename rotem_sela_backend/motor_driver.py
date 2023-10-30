#!/usr/bin/python

from PCA9685 import PCA9685
import time
from enum import Enum

Dir = [
    'forward',
    'reverse',
]

pwm0 = PCA9685(0x40, debug=False)
pwm0.setPWMFreq(50)
#pwm1 = PCA9685(0x41, debug=False)
#pwm1.setPWMFreq(50)


class RobotMotor(Enum):
    Head        = 1
    HeadTurn    = 2
    Tail        = 3
    TailTurn    = 4


class MotorDriver():
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    def MotorRun(self, motor, index, speed):
        if speed > 100 or speed<0 :
            return
        if motor == RobotMotor.Head:
            pwm0.setDutycycle(self.PWMA, speed)
            if(index == Dir[0]):
                print ("Head Forward")
                pwm0.setLevel(self.AIN1, 0)
                pwm0.setLevel(self.AIN2, 1)
            else:
                print ("Head Reverse")
                pwm0.setLevel(self.AIN1, 1)
                pwm0.setLevel(self.AIN2, 0)
        elif motor == RobotMotor.HeadTurn:
            pwm0.setDutycycle(self.PWMB, speed)
            if(index == Dir[0]):
                print ("HeadTurn Right")
                pwm0.setLevel(self.BIN1, 0)
                pwm0.setLevel(self.BIN2, 1)
            else:
                print ("HeadTurn Left ")
                pwm0.setLevel(self.BIN1, 1)
                pwm0.setLevel(self.BIN2, 0)
        # elif motor == RobotMotor.Tail:
        #     pwm1.setDutycycle(self.PWMA, speed)
        #     if(index == Dir[0]):
        #         print ("Tail Forward")
        #         pwm1.setLevel(self.AIN1, 0)
        #         pwm1.setLevel(self.AIN2, 1)
        #     else:
        #         print ("Tail Reverse")
        #         pwm1.setLevel(self.AIN1, 1)
        #         pwm1.setLevel(self.AIN2, 0)
        # elif motor == RobotMotor.TailTurn:
        #     pwm1.setDutycycle(self.PWMB, speed)
        #     if(index == Dir[0]):
        #         print ("TailTurn Right")
        #         pwm1.setLevel(self.BIN1, 0)
        #         pwm1.setLevel(self.BIN2, 1)
        #     else:
        #         print ("TailTurn Left ")
        #         pwm1.setLevel(self.BIN1, 1)
        #         pwm1.setLevel(self.BIN2, 0)

    def MotorStop(self, motor):
        if motor == RobotMotor.Head:
            print("Head Stop")
            pwm0.setDutycycle(self.PWMA, 0)
        elif motor == RobotMotor.HeadTurn:
            print("HeadTurn Stop")
            pwm0.setDutycycle(self.PWMB, 0)
        # elif motor == RobotMotor.Tail:
        #     print("Tail Stop")
        #     pwm1.setDutycycle(self.PWMA, 0)
        # elif motor == RobotMotor.TailTurn:
        #     print("TailTurn Stop")
        #     pwm1.setDutycycle(self.PWMB, 0)
 


if __name__ == "__main__":
    print("this is a motor driver test code")
    Motor = MotorDriver()
    
    Motor.MotorRun(RobotMotor.Head, 'forward', 20)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.Head)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.Head, 'reverse', 20)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.Head)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.HeadTurn, 'forward', 20)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.HeadTurn)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.HeadTurn, 'reverse', 20)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.HeadTurn)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.Tail, 'forward', 20)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.Tail)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.Tail, 'reverse', 20)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.Tail)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.TailTurn, 'forward', 20)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.TailTurn)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.TailTurn, 'reverse', 20)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.TailTurn)