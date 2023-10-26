import threading
import time
import queue
import logging
import os
import datetime

import pwm_rpi
import ps4_controller
import robot_remote_control
from enum import Enum


KEEP_ALIVE_TIMEOUT_SEC = 1.0

class CommandOpcode(Enum):
    motor = 1
    keep_alive = 2
    camera = 3
    telemetric = 4

RC = "REMOTE"  # RC=Remote Control

class RobotMain():
    def __init__(self) -> None:
        self.comm_thread = threading.Thread(target=self.CommRxHandle)
        self.rx_q = queue.Queue()
        self.tx_q = queue.Queue()
        if RC == "LOCAL":
            self.ps4Conroller = ps4_controller.RobotPS4(self.rx_q, interface="/dev/input/js0", connecting_using_ds4drv=False)
        else:
            self.comm = robot_remote_control.RobotRemoteControl(self.rx_q)
        self.main_thread = threading.Thread(target=self.RobotMain)
        self.keep_alive_thread = threading.Thread(target=self.TestKeepAlive)
        self.last_keep_alive = datetime.datetime.now()
        self.motors = pwm_rpi.MotorDriver()

        self.comm_thread.start()
        self.main_thread.start()
        self.keep_alive_thread.start()
       
        
        
    def CommRxHandle(self):
        if RC == "LOCAL":
            self.ps4Conroller.listen()
        else:
            while True:
                self.comm.start()
        
    def RobotMain(self):
        while True:
            event = self.rx_q.get(0.5)
            if event["opcode"] == CommandOpcode.motor.value:
                self.MotorHandler(event)
            if event["opcode"] == CommandOpcode.keep_alive.value:
                self.KeepAliveHandler()


    def MotorHandler(self,event):
        speed = event["value"]
        print(f"dutyc={speed}")
        motor = pwm_rpi.RobotMotor(event["motor"])
        dir = event["dir"]

        if speed < 10:
            self.motors.MotorStop(motor)
        else:           
            self.motors.MotorRun(motor,dir,speed)
        
    def KeepAliveHandler(self):
        self.last_keep_alive = datetime.datetime.now()
        self.motors.disable_motors = False
        self.TelemetricInfoSend()

    def TestKeepAlive(self):
        while True:
            delta = datetime.datetime.now() - self.last_keep_alive
            if delta.total_seconds() >= KEEP_ALIVE_TIMEOUT_SEC:
                self.motors.StopAllMotors()
            time.sleep(0.5) 

    def TelemetricInfoSend(self):
        info = {
            "opcode": CommandOpcode.telemetric,
            "imu-1" : [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
            "imu-2" : [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
            "drive1-CS" : 4096,
            "drive2-CS" : 4096,
            "turn1-CS"  : 4096,
            "turn2-CS"  : 4096,
            "elev-CS"   : 4096,
            "joint-CS"  : 4096,
            "FullTank1" : 4096,
            "FullTank2" : 4096,
            "FullTank3" : 4096,
            "Spare1"    : 4096,
            "Spare2"    : 4096,
            "Spare3"    : 4096,
            "Spare4"    : 4096,
            "Spare5"    : 4096,
            "Spare6"    : 4096,
            "Spare7"    : 4096,
            "Camera-F1" : True,
            "Camera-S1" : True,
            "Camera-F2" : True,
            "Camera-S2" : True,
            "Camera-F3" : True,
            "Camera-S3" : True,
            "Camera-F4" : True,
            "Camera-S4" : True
        }
        self.comm.Transmit(info)

if __name__ == "__main__":
    obj = RobotMain()