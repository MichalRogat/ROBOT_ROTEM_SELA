import threading
import time
import queue
import logging
import os

import pwm_rpi
import ps4_controller

class RobotMain():
    def __init__(self) -> None:
        logging.info("Init")
        self.ps4_thread = threading.Thread(target=self.PS4Handler)
        self.ps4_eventQ = queue.Queue()
        self.ps4Conroller = ps4_controller.RobotPS4(self.ps4_eventQ, interface="/dev/input/js0", connecting_using_ds4drv=False)
        
        self.mainthread = threading.Thread(target=self.RobotMain)
        self.ps4_thread.start()
        self.mainthread.start()
        self.motors = pwm_rpi.MotorDriver()
        
        
    def PS4Handler(self):
        self.ps4Conroller.listen(timeout=60)
        
    def RobotMain(self):
        while True:
            event = self.ps4_eventQ.get()
            speed = event["value"]
            motor = event["motor"]
            
            if motor.value < pwm_rpi.RobotMotor.Func1.value:
                dir = event["dir"]
                if speed < 20:
                    self.motors.MotorStop(motor)
                else:           
                    self.motors.MotorRun(motor,dir,speed)
            else:
                self.motors.SetRelay(motor, speed)
                
             
             
if __name__ == "__main__":
    obj = RobotMain()
    