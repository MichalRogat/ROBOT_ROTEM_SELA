import RPi.GPIO as GPIO
import time
from enum import Enum

PWMA            = 12  #PWM0 GPIO18
PWMA_DIR        = 16  #DIRA GPIO23
PWMA_EN         = 18  #A_EN GPIO24
PWMB            = 32  #PWM0 GPIO12
PWMB_DIR        = 38  #DIRB GPIO16
PWMB_EN         = 22  #B_EN GPIO25
PWMC            = 33  #PWM1 GPIO13
PWMC_DIR        = 29  #DIRC GPIO05
PWMC_EN         = 24  #C_EN GPIO10  sleep1 
PWMD            = 35  #PWM1 GPIO19
PWMD_DIR        = 31  #PWM1 GPIO06
PWMD_EN         = 13  #D_EN GPIO27
PUMP1_EN        = 15  #PUMP1_EN GPIO22        
PUMP2_EN        = 19  #PUMP2_EN GPIO10  
PUMP3_EN        = 21  #PUMP3_EN GPIO09  
PUMP4_EN        = 23  #PUMP4_EN GPIO11  
PUMP_DIR        = 37  #PUMP_DIR_EN GPIO26  

PWM0_FREQ = 5000
PWM1_FREQ = 5000

Dir = [
    'forward',
    'reverse',
]

class RobotMotor(Enum):
    Drive1      = 1
    Drive2      = 2
    Elev1       = 3
    Elev2       = 4
    Turn1       = 5
    Turn2       = 6
    Joint1      = 7
    Joint2      = 8
    Roller      = 9
    Pump1       = 10
    Pump2       = 11
    Pump3       = 12
    Pump4       = 13
    

class MotorDriver:
    def __init__(self):
        GPIO.cleanup()
        GPIO.setwarnings(False)			#disable warnings
        GPIO.setmode(GPIO.BOARD)		#set pin numbering system
        GPIO.setup(PWMA           , GPIO.OUT)
        GPIO.setup(PWMA_DIR       , GPIO.OUT)
        GPIO.setup(PWMA_EN        , GPIO.OUT)
        GPIO.setup(PWMB           , GPIO.OUT)
        GPIO.setup(PWMB_DIR       , GPIO.OUT) 
        GPIO.setup(PWMB_EN        , GPIO.OUT)
        GPIO.setup(PWMC           , GPIO.OUT) 
        GPIO.setup(PWMC_DIR       , GPIO.OUT)
        GPIO.setup(PWMC_EN        , GPIO.OUT)
        GPIO.setup(PWMD           , GPIO.OUT) 
        GPIO.setup(PWMD_DIR       , GPIO.OUT)
        GPIO.setup(PWMD_EN        , GPIO.OUT)
        # GPIO.setup(PWMC_ELEV2_EN  , GPIO.OUT)
        # GPIO.setup(PWMC_TURN1_EN  , GPIO.OUT)
        # GPIO.setup(PWMC_TURN2_EN  , GPIO.OUT)
        # GPIO.setup(PWMC_JOINT1_EN , GPIO.OUT) 
        # GPIO.setup(PWMC_JOINT2_EN , GPIO.OUT)
        # GPIO.setup(PWMD           , GPIO.OUT) 
        # GPIO.setup(PWMD_DIR       , GPIO.OUT)
        # GPIO.setup(PWMD_EN        , GPIO.OUT)
        # GPIO.setup(PUMP1_EN       , GPIO.OUT) 
        # GPIO.setup(PUMP2_EN       , GPIO.OUT)
        # GPIO.setup(PUMP3_EN       , GPIO.OUT) 
        # GPIO.setup(PUMP4_EN       , GPIO.OUT)
        # GPIO.setup(PUMP_DIR       , GPIO.OUT)     
        
        self.pwmA = GPIO.PWM(PWMA, PWM0_FREQ )
        self.pwmA.start(0)
        self.pwmB = GPIO.PWM(PWMB, PWM0_FREQ)
        self.pwmB.start(0)
        self.pwmC = GPIO.PWM(PWMC, PWM1_FREQ)
        self.pwmC.start(0)
        self.pwmD = GPIO.PWM(PWMD, PWM1_FREQ)
        self.pwmD.start(0)
        # self.lastPwmCMotor = RobotMotor.Elev1
        # self.lastPumpMotor = RobotMotor.Pump1
    
    def DisableMuxPWMC(self):
        print("Disable PWMC")
        # GPIO.output(PWMC_ELEV1_EN  , GPIO.LOW)
        # GPIO.output(PWMC_ELEV2_EN  , GPIO.LOW)
        # GPIO.output(PWMC_TURN1_EN  , GPIO.LOW)
        # GPIO.output(PWMC_TURN2_EN  , GPIO.LOW)
        # GPIO.output(PWMC_JOINT1_EN , GPIO.LOW) 
        # GPIO.output(PWMC_JOINT2_EN , GPIO.LOW)
        
    def DisablePumps(self):
        print("Disable Pumps")
        GPIO.output(PUMP1_EN  , GPIO.LOW)
        GPIO.output(PUMP2_EN  , GPIO.LOW)
        GPIO.output(PUMP3_EN  , GPIO.LOW)
        GPIO.output(PUMP4_EN  , GPIO.LOW)
            
        
    def MotorRun(self, motor, index, speed):
        if speed > 100 or speed<0 :
            return
        if motor == RobotMotor.Drive1:
            #GPIO.output(PWMA_EN, GPIO.HIGH)
            self.pwmA.ChangeDutyCycle(speed)
            if(index == Dir[0]):
                print ("Drive1 Forward")
                GPIO.output(PWMA_EN, GPIO.LOW)
                GPIO.output(PWMA_DIR, GPIO.HIGH)
            else:
                print ("Drive1 Reverse")
                GPIO.output(PWMA_EN, GPIO.HIGH)
                GPIO.output(PWMA_DIR, GPIO.LOW)
        elif motor == RobotMotor.Drive2:
            #GPIO.output(PWMB_EN, GPIO.HIGH)
            self.pwmB.ChangeDutyCycle(speed)
            if(index == Dir[0]):
                print ("Drive2 Forward")
                GPIO.output(PWMB_EN, GPIO.LOW)
                GPIO.output(PWMB_DIR, GPIO.HIGH)
            else:
                print ("Drive2 Reverse ")
                GPIO.output(PWMB_EN, GPIO.HIGH)
                GPIO.output(PWMB_DIR, GPIO.LOW)
        elif motor == RobotMotor.Turn1:
            #GPIO.output(PWMD_EN, GPIO.HIGH)
            self.pwmC.ChangeDutyCycle(speed)
            if(index == Dir[0]):
                print ("Roller Forward")
                GPIO.output(PWMC_EN, GPIO.LOW)
                GPIO.output(PWMC_DIR, GPIO.HIGH)
            else:
                print ("Roller Reverse")
                GPIO.output(PWMC_EN, GPIO.HIGH)
                GPIO.output(PWMC_DIR, GPIO.LOW)
        elif motor == RobotMotor.Turn2:
            #GPIO.output(PWMD_EN, GPIO.HIGH)
            self.pwmD.ChangeDutyCycle(speed)
            if(index == Dir[0]):
                print ("Roller Forward")
                GPIO.output(PWMD_EN, GPIO.LOW)
                GPIO.output(PWMD_DIR, GPIO.HIGH)
            else:
                print ("Roller Reverse")
                GPIO.output(PWMD_EN, GPIO.HIGH)
                GPIO.output(PWMD_DIR, GPIO.LOW)
         
    def MotorStop(self, motor):
        if motor == RobotMotor.Drive1:
            print("Drive1 Stop")
            GPIO.output(PWMA_EN, GPIO.LOW)
            GPIO.output(PWMA_DIR, GPIO.LOW)
            self.pwmA.ChangeDutyCycle(0)
        elif motor == RobotMotor.Drive2:
            print("Drive2 Stop")
            GPIO.output(PWMB_EN, GPIO.LOW)
            GPIO.output(PWMB_DIR, GPIO.LOW)
            self.pwmB.ChangeDutyCycle(0)
        elif motor == RobotMotor.Turn1:
            print("Elev1 Stop")
            GPIO.output(PWMC_EN, GPIO.LOW)
            GPIO.output(PWMC_DIR, GPIO.LOW)
            self.pwmC.ChangeDutyCycle(0)
        elif motor == RobotMotor.Turn2:
            print("Joint2 Stop")
            GPIO.output(PWMD_EN, GPIO.LOW)
            GPIO.output(PWMD_DIR, GPIO.LOW)
            self.pwmD.ChangeDutyCycle(0)
       

if __name__ == "__main__":
    print("this is a motor driver test code")
    while True:
        Motor = MotorDriver()
        Motor.MotorRun(RobotMotor.Drive1, 'forward', 50)
        time.sleep(1)
        Motor.MotorStop(RobotMotor.Drive1)
        time.sleep(1)
        Motor.MotorRun(RobotMotor.Drive1, 'reverse', 50)
        time.sleep(1)
        Motor.MotorStop(RobotMotor.Drive1)
        time.sleep(1)
        Motor.MotorRun(RobotMotor.Drive2, 'forward', 50)
        time.sleep(1)
        Motor.MotorStop(RobotMotor.Drive2)
        time.sleep(1)
        Motor.MotorRun(RobotMotor.Drive2, 'reverse', 50)
        time.sleep(1)
        Motor.MotorStop(RobotMotor.Drive2)
        time.sleep(1)
        
