# Maintainer: Dorin

import RPi.GPIO as GPIO
from minimu import MinIMU_v5_pi
import threading
        
class MotorDrive:
    IN1 = 32 # GPIO 12 PWM0
    IN2 = 33 # GPIO 13 PWM1
    PMODE = 31 # GPIO 6 set high to enable IN1/IN2 mode
    PWM0_FREQ = 5000

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BOARD)		#set pin numbering system
        GPIO.setup(MotorDrive.PMODE , GPIO.OUT)
        GPIO.setup(MotorDrive.IN1, GPIO.OUT)
        GPIO.setup(MotorDrive.IN2, GPIO.OUT)

        GPIO.output(MotorDrive.PMODE, GPIO.HIGH) # GPIO 6 set high to enable IN1/IN2 mode

        self.pwmA = None
        self.pwmB = None
        
    def moveClockWise(self):
        if self.pwmA is None:
            self.pwmA = GPIO.PWM(MotorDrive.IN1, MotorDrive.PWM0_FREQ)
            GPIO.output(MotorDrive.IN2, GPIO.HIGH)
            self.pwmB = None
        self.pwmA.start(90)
        

    def moveCounterClock(self):
        if self.pwmB is None:
            self.pwmB = GPIO.PWM(MotorDrive.IN2, MotorDrive.PWM0_FREQ)
            GPIO.output(MotorDrive.IN1, GPIO.HIGH)
            self.pwmA = None
        self.pwmB.start(90)
        

    def stopMotor(self):
        if self.pwmA is not None:
            self.pwmA.stop()
        if self.pwmB is not None:
            self.pwmB.stop()
        GPIO.cleanup()

desired_role = 0.0
Kp = 1.0

def calculate_proportional(current_role):
    error = desired_role - current_role
    return Kp * error

def start_auto_drive():
    i2c_lock = threading.Lock()
    imu = MinIMU_v5_pi(1, i2c_lock)
    imu.trackAngle()
    while True:
        print(current_role := imu.prevAngle[0][0])
        pid_correction = calculate_proportional(current_role)
        if (pid_correction>=0):
            motor.moveClockWise()  
        else: 
            motor.moveCounterClock()

if __name__=="__main__":
    try:
        motor = MotorDrive()
        start_auto_drive()
    except KeyboardInterrupt:
        motor.stopMotor()
    finally:
        motor.stopMotor()
    
