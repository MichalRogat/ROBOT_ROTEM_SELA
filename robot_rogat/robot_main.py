import threading
import time
import queue
import logging
import os

import pwm_rpi
import ps4_controller
import robot_keyboard

class RobotMain():
    def __init__(self) -> None:
        logging.info("Init")
        self.ps4_thread = threading.Thread(target=self.PS4Handler)
        self.ps4_eventQ = queue.Queue()
        #self.ps4Conroller = ps4_controller.RobotPS4(self.ps4_eventQ, interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.robot_keyboard = robot_keyboard.RobotKeyboard(self.ps4_eventQ)
        self.mainthread = threading.Thread(target=self.RobotMain)
        self.ps4_thread.start()
        self.mainthread.start()
        self.motors = pwm_rpi.MotorDriver()
        
        
    def PS4Handler(self):
        #self.ps4Conroller.listen(timeout=60)
        self.robot_keyboard.listen()
        
    def RobotMain(self):
        while True:
            event = self.ps4_eventQ.get()
            speed = event["value"]
            print(f"dutyc={speed}")
            motor = event["motor"]
            dir = event["dir"]
            
            if motor.value < pwm_rpi.RobotMotor.Pump1.value:
               
                if speed < 10:
                    self.motors.MotorStop(motor)
                else:           
                    self.motors.MotorRun(motor,dir,speed)
            else:
                if speed == 1:
                    self.motors.MotorRun(motor,dir, speed)
                else:
                    self.motors.DisablePumps()
                    
if __name__ == "__main__":
    obj = RobotMain()
    