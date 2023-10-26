import threading
import time
import queue
import logging
import os

import pwm_rpi
import ps4_controller
import robot_remote_control
from enum import Enum

class CommandOpcode(Enum):
    motor = 1

RC = "REMOTE"  # RC=Remote Control

class RobotMain():
    def __init__(self) -> None:
        logging.info("Init")

        self.comm_thread = threading.Thread(target=self.CommRxHandle)
        self.rx_q = queue.Queue()
        self.tx_q = queue.Queue()
        if RC == "LOCAL":
            self.ps4Conroller = ps4_controller.RobotPS4(self.rx_q, interface="/dev/input/js0", connecting_using_ds4drv=False)
        else:
            self.comm = robot_remote_control.RobotRemoteControl(self.rx_q)
        self.main_thread = threading.Thread(target=self.RobotMain)
        self.comm_thread.start()
        self.main_thread.start()
        self.motors = pwm_rpi.MotorDriver()
        
        
    def CommRxHandle(self):
        if RC == "LOCAL":
            self.ps4Conroller.listen()
        else:
            while True:
                self.comm.start()
        
    def RobotMain(self):
        while True:
            event = self.rx_q.get()
            if event["opcode"] == CommandOpcode.motor.value:
                self.MotorHandler(event)

    def MotorHandler(self,event):
        speed = event["value"]
        print(f"dutyc={speed}")
        motor = pwm_rpi.RobotMotor(event["motor"])
        dir = event["dir"]
        
        if speed < 10:
            self.motors.MotorStop(motor)
        else:           
            self.motors.MotorRun(motor,dir,speed)

if __name__ == "__main__":
    obj = RobotMain()