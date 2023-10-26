import sys 
import threading
import time
import queue
import logging
import os
import ps4_usb
import robot_comm
from pwm_rpi import RobotMotor

class RobotRemote():
    def __init__(self) -> None:

        self.rx_q = queue.Queue()
        self.comm = robot_comm.RobotComm(self.rx_q)
        self.remote = ps4_usb.PS4Controller(self.rx_q)

        self.comm_thread = threading.Thread(target=self.CommRxHandler)
        self.ps4_thread  = threading.Thread(target=self.Ps4Handler)
        self.main_thread = threading.Thread(target=self.RobotMain)

        self.comm_thread.start()
        self.main_thread.start()
        self.ps4_thread.start()
    
    def Ps4Handler(self):
        self.remote.listen()

    def CommRxHandler(self):
        self.comm.RxHandler()
        
    def RobotMain(self):
        while True:
            event = self.rx_q.get()
            speed = event["value"]
            motor = event["motor"]
            dir = event["dir"]
            if motor.value <  RobotMotor.Roller.value:
                if speed > 5:
                    self.comm.Transmit(event)
            else:
                self.comm.Transmit(event)

                
if __name__ == "__main__":
    obj = RobotRemote()