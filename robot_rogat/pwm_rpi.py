import RPi.GPIO as GPIO
import time
from enum import Enum

MOTORA      = 12  #PWM0 GPIO18
MOTORA_DIR  = 16  #DIRA GPIO23
MOTORB      = 32  #PWM0 GPIO12
MOTORB_DIR  = 36  #DIRB GPIO12
MOTORC      = 33  #PWM1 GPIO33
MOTORC_DIR  = 29  #DIRC GPIO06
MOTORD      = 35  #PWM1 GPIO19
MOTORD_DIR  = 31  #PWM1 GPIO26

RELAY1      = 37
RELAY2      = 38
RELAY3      = 40

PWM0_FREQ = 5000
PWM1_FREQ = 5000

Dir = [
    'forward',
    'reverse',
]

class RobotMotor(Enum):
    Head        = 1
    HeadTurn    = 2
    Tail        = 3
    TailTurn    = 4
    Func1       = 5
    Func2       = 6
    Func3       = 7
    

class MotorDriver:
    def __init__(self):
        GPIO.cleanup()
        GPIO.setwarnings(False)			#disable warnings
        GPIO.setmode(GPIO.BOARD)		#set pin numbering system
        GPIO.setup(MOTORA, GPIO.OUT)
        GPIO.setup(MOTORA_DIR, GPIO.OUT)
        GPIO.setup(MOTORB, GPIO.OUT)
        GPIO.setup(MOTORB_DIR, GPIO.OUT) 
        GPIO.setup(MOTORC, GPIO.OUT)
        GPIO.setup(MOTORC_DIR, GPIO.OUT) 
        GPIO.setup(MOTORD, GPIO.OUT)
        GPIO.setup(MOTORD_DIR, GPIO.OUT)
        
        GPIO.setup(RELAY1, GPIO.OUT)
        GPIO.setup(RELAY2, GPIO.OUT)
        GPIO.setup(RELAY3, GPIO.OUT)
        
        
        self.motorA = GPIO.PWM(MOTORA, PWM0_FREQ )
        self.motorA.start(0)
        self.motorB = GPIO.PWM(MOTORB, PWM0_FREQ)
        self.motorB.start(0)
        self.motorC = GPIO.PWM(MOTORC, PWM1_FREQ)
        self.motorC.start(0)
        self.motorD = GPIO.PWM(MOTORD, PWM1_FREQ)
        self.motorD.start(0)
        
         
    def ChangeDutyCycle(self, motor,value):
        if motor == RobotMotor.Front:
            self.motorA.ChangeDutyCycle(value)
        elif motor == RobotMotor.Mid:
            self.motorB.ChangeDutyCycle(value)
        elif motor == RobotMotor.Rear:
            self.motorC.ChangeDutyCycle(value)
        elif motor == RobotMotor.Unknow:
            self.motorD.ChangeDutyCycle(value)
    
    def MotorRun(self, motor, index, speed):
        if speed > 100 or speed<0 :
            return
        if motor == RobotMotor.Head:
            self.motorA.ChangeDutyCycle(speed)
            if(index == Dir[0]):
                print ("Head Forward")
                GPIO.output(MOTORA_DIR, GPIO.HIGH)
            else:
                print ("Head Reverse")
                GPIO.output(MOTORA_DIR, GPIO.LOW)
        elif motor == RobotMotor.HeadTurn:
            self.motorB.ChangeDutyCycle(speed)
            if(index == Dir[0]):
                print ("HeadTurn Right")
                GPIO.output(MOTORB_DIR, GPIO.HIGH)
            else:
                print ("HeadTurn Left ")
                GPIO.output(MOTORB_DIR, GPIO.LOW)
        elif motor == RobotMotor.Tail:
            self.motorC.ChangeDutyCycle(speed)
            if(index == Dir[0]):
                print ("Tail Forward")
                GPIO.output(MOTORC_DIR, GPIO.HIGH)
            else:
                print ("Tail Reverse")
                GPIO.output(MOTORC_DIR, GPIO.LOW)
        elif motor == RobotMotor.TailTurn:
            self.motorD.ChangeDutyCycle(speed)
            if(index == Dir[0]):
                print ("TailTurn Right")
                GPIO.output(MOTORD_DIR, GPIO.HIGH)
            else:
                print ("TailTurn Left ")
                GPIO.output(MOTORD, GPIO.LOW)
    
    def SetRelay(self, relay, value):
        if relay == RobotMotor.Func1:
            if value == 1:
                print ("Func1 High")
                GPIO.output(RELAY1, GPIO.HIGH)
            else:
                print ("Func1 Low")
                GPIO.output(RELAY1, GPIO.LOW)
        elif relay == RobotMotor.Func2:
            if value == 1:
                print ("Func2 High")
                GPIO.output(RELAY2, GPIO.HIGH)
            else:
                print ("Func2 Low")
                GPIO.output(RELAY2, GPIO.LOW)
        elif relay == RobotMotor.Func3:
            if value == 1:
                print ("Func3 High")
                GPIO.output(RELAY3, GPIO.HIGH)
            else:
                print ("Func3 Low")
                GPIO.output(RELAY3, GPIO.LOW)
                

    def MotorStop(self, motor):
        if motor == RobotMotor.Head:
            print("Head Stop")
            self.motorA.ChangeDutyCycle(0)
        elif motor == RobotMotor.HeadTurn:
            print("HeadTurn Stop")
            self.motorB.ChangeDutyCycle(0)
        elif motor == RobotMotor.Tail:
            print("Tail Stop")
            self.motorC.ChangeDutyCycle(0)
        elif motor == RobotMotor.TailTurn:
            print("TailTurn Stop")
            self.motorD.ChangeDutyCycle(0)

if __name__ == "__main__":
    print("this is a motor driver test code")
    Motor = MotorDriver()
    Motor.MotorRun(RobotMotor.Head, 'forward', 80)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.Head)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.Head, 'reverse', 80)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.Head)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.HeadTurn, 'forward', 80)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.HeadTurn)
    time.sleep(1)
    Motor.MotorRun(RobotMotor.HeadTurn, 'reverse', 80)
    time.sleep(1)
    Motor.MotorStop(RobotMotor.HeadTurn)
    # time.sleep(1)
    # Motor.MotorRun(RobotMotor.Tail, 'forward', 80)
    # time.sleep(1)
    # Motor.MotorStop(RobotMotor.Tail)
    # time.sleep(1)
    # Motor.MotorRun(RobotMotor.Tail, 'reverse', 80)
    # time.sleep(1)
    # Motor.MotorStop(RobotMotor.Tail)
    # time.sleep(1)
    # Motor.MotorRun(RobotMotor.TailTurn, 'forward', 80)
    # time.sleep(1)
    # Motor.MotorStop(RobotMotor.TailTurn)
    # time.sleep(1)
    # Motor.MotorRun(RobotMotor.TailTurn, 'reverse', 80)
    # time.sleep(1)
    # Motor.MotorStop(RobotMotor.TailTurn)
    time.sleep(1)
    Motor.SetRelay(RobotMotor.Func1,0)
    time.sleep(1)
    Motor.SetRelay(RobotMotor.Func1,1)
    time.sleep(1)
    Motor.SetRelay(RobotMotor.Func2,0)
    time.sleep(1)
    Motor.SetRelay(RobotMotor.Func2,1)
    time.sleep(1)
    Motor.SetRelay(RobotMotor.Func3,0)
    time.sleep(1)
    Motor.SetRelay(RobotMotor.Func3,1)