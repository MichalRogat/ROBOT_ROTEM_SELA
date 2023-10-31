import RPi.GPIO as GPIO
import threading
import time
import datetime
from enum import Enum

PWMA            = 12  #PWM0 GPIO18
PWMA_DIR        = 16  #DIRA GPIO23
PWMA_EN         = 18  #A_EN GPIO24
PWMB            = 32  #PWM0 GPIO12
PWMB_DIR        = 36  #DIRB GPIO16
PWMB_EN         = 22  #B_EN GPIO25
PWMC            = 33  #PWM1 GPIO13
PWMC_DIR        = 11  #DIRC GPIO05
PWMC_EN         = 38  #C_EN GPIO10  sleep1 
TURN1_EN         = 24
TURN1_DIR        = 11 #GPIO 17
PWMD            = 35  #PWM1 GPIO19
PWMD_DIR        = 13  #PWM1 GPIO06
JOINT1_EN       = 7         
PWMD_EN         = 40  #D_EN GPIO27
PUMP1_EN        = 15  #PUMP1_EN GPIO22        
PUMP2_EN        = 19  #PUMP2_EN GPIO10  
PUMP3_EN        = 21  #PUMP3_EN GPIO09  
PUMP4_EN        = 23  #PUMP4_EN GPIO11  
PUMP_DIR        = 37  #PUMP_DIR_EN GPIO26  

PWM0_FREQ = 5000
PWM1_FREQ = 5000
MOTORS_TO_MONITOR = 6
TIME_TO_DISABLE_MOTOR_OVERCURRENT_SEC= 1.0

class RobotMotor(Enum):
    Drive1      = 0
    Drive2      = 1
    Elev1       = 2
    Elev2       = 3
    Turn1       = 4
    Turn2       = 5
    Joint1      = 6
    Joint2      = 7
    Roller      = 8
    Pump1       = 9
    Pump2       = 10
    Pump3       = 11
    Pump4       = 12
    

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
        GPIO.setup(TURN1_EN  , GPIO.OUT)
        GPIO.setup(TURN1_DIR  , GPIO.OUT)
        GPIO.setup(JOINT1_EN  , GPIO.OUT)
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

        self.disable_motors = False
        self.over_current = [False] * (RobotMotor.Pump4.value+1)
        self.motor_speed = [0] * (RobotMotor.Pump4.value+1)
        self.current_limit = [1.0] * (RobotMotor.Pump4.value+1)
        self.motor_current = [0] * (RobotMotor.Pump4.value+1)

    def StopAllMotors(self):
        if not self.disable_motors:
            print("Disable all motors")
            self.pwmA.ChangeDutyCycle(0)
            self.pwmB.ChangeDutyCycle(0)
            self.pwmC.ChangeDutyCycle(0)
            self.pwmD.ChangeDutyCycle(0)
            GPIO.output(PWMA_EN, GPIO.LOW)
            GPIO.output(PWMB_EN, GPIO.LOW)
            GPIO.output(PWMC_EN, GPIO.LOW)
            GPIO.output(TURN1_EN, GPIO.LOW)
            GPIO.output(JOINT1_EN, GPIO.LOW)
            GPIO.output(PWMD_EN, GPIO.LOW)
            GPIO.output(PUMP1_EN, GPIO.LOW)
            GPIO.output(PUMP2_EN, GPIO.LOW)
            GPIO.output(PUMP3_EN, GPIO.LOW)
            GPIO.output(PUMP4_EN, GPIO.LOW)            
        self.disable_motors = True

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
            
        
    def MotorRun(self, motor, speed):
        if speed > 100 or speed < -100:
            return
        if speed>=0:
            dir='f'
        else:
            dir ='r'
        speed=abs(speed)

        if motor == RobotMotor.Drive1:
            #GPIO.output(PWMA_EN, GPIO.HIGH)
            self.motor_speed[RobotMotor.Drive1.value]=speed
            # if self.over_current[RobotMotor.Drive1.value]:
            #     return
            GPIO.output(PWMA_EN, GPIO.HIGH)
            if dir=='f':
                print ("Drive1 Forward")
                self.pwmA.ChangeDutyCycle(100-speed)
                GPIO.output(PWMA_DIR, GPIO.HIGH)
            else:
                print ("Drive1 Reverse")
                self.pwmA.ChangeDutyCycle(speed)
                GPIO.output(PWMA_DIR, GPIO.LOW)

        elif motor == RobotMotor.Drive2:
            #GPIO.output(PWMB_EN, GPIO.HIGH)
            self.motor_speed[RobotMotor.Drive2.value]=speed
            GPIO.output(PWMB_EN, GPIO.HIGH)
            if dir=='f':
                print ("Drive2 Forward")
                self.pwmB.ChangeDutyCycle(100-speed)
                GPIO.output(PWMB_DIR, GPIO.HIGH)
            else:
                print ("Drive2 Reverse ")
                self.pwmB.ChangeDutyCycle(speed)
                GPIO.output(PWMB_DIR, GPIO.LOW)

        elif motor == RobotMotor.Turn1:
            #GPIO.output(PWMD_EN, GPIO.HIGH)
            self.motor_speed[RobotMotor.Turn1.value]=speed
            GPIO.output(TURN1_EN, GPIO.HIGH)
            if dir=='f':
                print ("Turn1 Right")
                self.pwmC.ChangeDutyCycle(abs(100-speed))
                GPIO.output(TURN1_DIR, GPIO.HIGH)
            else:
                print ("Turn1 Left")
                self.pwmC.ChangeDutyCycle(speed)
                GPIO.output(TURN1_DIR, GPIO.LOW)

        elif motor == RobotMotor.Elev1:
            #GPIO.output(PWMD_EN, GPIO.HIGH)
            self.motor_speed[RobotMotor.Elev1.value]=speed
            GPIO.output(PWMC_EN, GPIO.HIGH)
            if dir=='f':
                print ("Elev1 Forward")
                self.pwmC.ChangeDutyCycle(abs(100-speed))
                GPIO.output(PWMC_DIR, GPIO.HIGH)
            else:
                print ("Elev1 Reverse")
                self.pwmC.ChangeDutyCycle(speed)
                GPIO.output(PWMC_DIR, GPIO.LOW) 

        elif motor == RobotMotor.Turn2:
            #GPIO.output(PWMD_EN, GPIO.HIGH)
            self.motor_speed[RobotMotor.Turn2.value]=speed
            GPIO.output(PWMD_EN, GPIO.HIGH)
            if dir=='f':
                print ("Turn2 Right")
                self.pwmD.ChangeDutyCycle(abs(100-speed))
                GPIO.output(PWMD_DIR, GPIO.HIGH)
            else:
                print ("Turn2 Left")
                self.pwmD.ChangeDutyCycle(speed)
                GPIO.output(PWMD_DIR, GPIO.LOW)

        elif motor == RobotMotor.Joint1:
            #GPIO.output(PWMD_EN, GPIO.HIGH)
            self.motor_speed[RobotMotor.Joint1.value]=speed
            GPIO.output(JOINT1_EN, GPIO.HIGH)
            if dir=='f':
                #print ("Joint1 Forward")
                self.pwmD.ChangeDutyCycle(abs(100-speed))
                GPIO.output(PWMD_DIR, GPIO.HIGH)
            else:
                #print ("Joint1 Reverse")
                self.pwmD.ChangeDutyCycle(speed)
                GPIO.output(PWMD_DIR, GPIO.LOW)
         
    def MotorStop(self, motor):
        if motor == RobotMotor.Drive1:
            print("Drive1 Stop")
            self.motor_speed[RobotMotor.Drive1.value]=0
            GPIO.output(PWMA_EN, GPIO.LOW)
            GPIO.output(PWMA_DIR, GPIO.LOW)
            self.pwmA.ChangeDutyCycle(0)

            print("Drive2 Stop")
            self.motor_speed[RobotMotor.Drive2.value]=0
            GPIO.output(PWMB_EN, GPIO.LOW)
            GPIO.output(PWMB_DIR, GPIO.LOW)
            self.pwmB.ChangeDutyCycle(0)
        if motor == RobotMotor.Drive2:
            print("Drive2 Stop")
            self.motor_speed[RobotMotor.Drive2.value]=0
            GPIO.output(PWMB_EN, GPIO.LOW)
            GPIO.output(PWMB_DIR, GPIO.LOW)
            self.pwmB.ChangeDutyCycle(0)
        if motor == RobotMotor.Elev1:
            print("Elev1 Stop")
            self.motor_speed[RobotMotor.Elev1.value]=0
            GPIO.output(PWMC_EN, GPIO.LOW)
            GPIO.output(PWMC_DIR, GPIO.LOW)
            self.pwmC.ChangeDutyCycle(0)
        if motor == RobotMotor.Turn1:
            print("Turn1 Stop")
            self.motor_speed[RobotMotor.Turn1.value]=0
            GPIO.output(PWMC_EN, GPIO.LOW)
            GPIO.output(PWMC_DIR, GPIO.LOW)
            self.pwmC.ChangeDutyCycle(0)
        if motor == RobotMotor.Joint1:
            print("Joint1 Stop")
            self.motor_speed[RobotMotor.Joint1.value]=0
            GPIO.output(PWMD_EN, GPIO.LOW)
            GPIO.output(PWMD_DIR, GPIO.LOW)
            self.pwmD.ChangeDutyCycle(0)
        if motor == RobotMotor.Turn2:
            print("Turn2 Stop")
            self.motor_speed[RobotMotor.Turn2.value]=0
            GPIO.output(PWMD_EN, GPIO.LOW)
            GPIO.output(PWMD_DIR, GPIO.LOW)
            self.pwmD.ChangeDutyCycle(0)

    def MotorTestCurrentOverload(self,a2d_values):
        #in case overload detected we disable the motor pwm for 1 sec
        for i in range(0,RobotMotor.Joint2.value):
            if self.motor_speed[i] > 0:
                self.motor_current[i] = a2d_values[i]
                if not self.over_current[i] and self.motor_current[i]>self.current_limit[i]:
                    print(f"{RobotMotor(i)} OverCurrent")
                    if RobotMotor.Drive1.value == i:
                        self.pwmA.ChangeDutyCycle(0)
                    if RobotMotor.Drive2.value == i:
                        self.pwmB.ChangeDutyCycle(0)
                    if RobotMotor.Turn1.value == i or RobotMotor.Elev1.value == i:
                        self.pwmC.ChangeDutyCycle(0)
                    if RobotMotor.Turn2.value == i or RobotMotor.Joint1.value == i:
                        self.pwmD.ChangeDutyCycle(0)
                    
                    self.over_current[i] = True
                    threading.Timer(TIME_TO_DISABLE_MOTOR_OVERCURRENT_SEC, self.OverCurrentDueTime,[i]).start()
                    
    def OverCurrentDueTime(self,motor):
        print(f"{RobotMotor(motor)} OverCurrent Relief Finished")
        self.over_current[motor] = False

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
        
